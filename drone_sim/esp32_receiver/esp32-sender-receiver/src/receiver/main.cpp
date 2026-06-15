// =============================================================================
// telemetry_receiver.cpp — ESP32 #2 (Récepteur)
//
// Rôle : reçoit la télémétrie JSON via UDP WiFi depuis l'ESP32 #1,
//        et affiche toutes les infos de vol sur l'OLED.
//
// Schéma d'affichage OLED :
//   Ligne 0  : Mode de vol + Batterie %
//   Ligne 1  : Altitude (grand)
//   Ligne 2  : VZ + Throttle
//   Ligne 3  : Roll / Pitch
//   Ligne 4  : Barres moteurs M0..M3
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>   // v6

// ── OLED ──────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDR  0x3C
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── WIFI / UDP ─────────────────────────────────────────────────────────────────
const char* ssid     = "ESP32_RESEAU";
const char* password = "12345678";
const int   port_udp = 1234;

WiFiUDP udp;
char buffer[512];

// ── LED STATUS (optionnelle, juste la LED interne) ────────────────────────────
#define LED_STATUS 2

// ── ÉTAT ──────────────────────────────────────────────────────────────────────
struct Telemetrie {
  float alt        = 0.0f;
  float vz         = 0.0f;
  float roll       = 0.0f;
  float pitch      = 0.0f;
  float yaw        = 0.0f;
  float bat_pct    = 100.0f;
  float bat_v      = 12.6f;
  float moteurs[4] = {0, 0, 0, 0};
  int   mode       = 0;
  float throttle   = 0.0f;
  float dist       = 0.0f;
  bool  valide     = false;
};

Telemetrie telem;
unsigned long dernierPaquet = 0;
unsigned long paquetsRecus  = 0;

// ── Utilitaires ───────────────────────────────────────────────────────────────
const char* nomMode(int m) {
  switch(m) {
    case 0: return "SOL";
    case 1: return "DECOLLE";
    case 2: return "VOL";
    case 3: return "ATTERRO";
    case 4: return "URGENCE!";
    default: return "?";
  }
}

// ── Parse JSON ────────────────────────────────────────────────────────────────
bool parseJSON(const char* json) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) return false;

  telem.alt        = doc["alt"]      | 0.0f;
  telem.vz         = doc["vz"]       | 0.0f;
  telem.roll       = doc["roll"]     | 0.0f;
  telem.pitch      = doc["pitch"]    | 0.0f;
  telem.yaw        = doc["yaw"]      | 0.0f;
  telem.bat_pct    = doc["bat_pct"]  | 100.0f;
  telem.bat_v      = doc["bat_v"]    | 12.6f;
  telem.moteurs[0] = doc["m0"]       | 0.0f;
  telem.moteurs[1] = doc["m1"]       | 0.0f;
  telem.moteurs[2] = doc["m2"]       | 0.0f;
  telem.moteurs[3] = doc["m3"]       | 0.0f;
  telem.mode       = doc["mode"]     | 0;
  telem.throttle   = doc["throttle"] | 0.0f;
  telem.dist       = doc["dist"]     | 0.0f;
  telem.valide     = true;
  return true;
}

// ── OLED affichage ────────────────────────────────────────────────────────────
void afficherOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  char buf[32];

  // ── Écran d'attente ──────────────────────────────────────────────────────
  if (!telem.valide) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("ESP32 #2 RECEPTEUR");
    display.setCursor(0, 14);
    display.println("Attente donnees...");
    unsigned long t = millis() / 1000;
    snprintf(buf, sizeof(buf), "Up: %lus", t);
    display.setCursor(0, 28);
    display.println(buf);
    display.display();
    return;
  }

  // ── Ligne 0 : Mode + Batterie ────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Barre graphique batterie (28px)
  int batPx = (int)(telem.bat_pct / 100.0f * 28.0f);
  batPx = max(0, min(28, batPx));
  display.drawRect(96, 1, 30, 7, SSD1306_WHITE);
  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);   // bout batterie
  display.fillRect(97, 2, batPx, 5, SSD1306_WHITE);

  snprintf(buf, sizeof(buf), "%-8s %3.0f%%", nomMode(telem.mode), telem.bat_pct);
  display.println(buf);

  // ── Ligne 1 : Altitude (grand texte) ────────────────────────────────────
  display.setCursor(0, 10);
  display.setTextSize(2);
  snprintf(buf, sizeof(buf), "%5.1f m", telem.alt);
  display.println(buf);

  // ── Ligne 2 : VZ + Throttle ─────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 28);
  snprintf(buf, sizeof(buf), "VZ%+5.2f THR%3.0f%%",
           telem.vz, telem.throttle * 100.0f);
  display.println(buf);

  // ── Ligne 3 : Roll / Pitch en degrés ────────────────────────────────────
  display.setCursor(0, 38);
  snprintf(buf, sizeof(buf), "R%+5.1fd P%+5.1fd",
           telem.roll  * 57.295f,
           telem.pitch * 57.295f);
  display.println(buf);

  // ── Ligne 4 : Barres moteurs M0(FL) M1(FR) M2(BL) M3(BR) ────────────────
  // Disposition vue de dessus :
  //   FL ─── FR
  //   BL ─── BR
  const char* labels[4] = {"FL", "FR", "BL", "BR"};
  int xCursor = 0;
  for (int i = 0; i < 4; i++) {
    display.setCursor(xCursor, 50);
    display.print(labels[i]);

    // Pourcentage moteur sous le label
    int pct = (int)(telem.moteurs[i] * 100.0f);
    snprintf(buf, sizeof(buf), "%3d%%", pct);
    display.setCursor(xCursor, 57);
    display.print(buf);

    // Barre graphique verticale (4px large, max 6px haut)
    int h = (int)(telem.moteurs[i] * 6.0f);
    h = max(0, min(6, h));
    display.fillRect(xCursor + 10, 63 - h, 4, h, SSD1306_WHITE);

    xCursor += 32;
  }

  display.display();
}

// ── SETUP ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    Serial.println("OLED non trouvé !");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 #2");
  display.setCursor(0, 12);
  display.println("Connexion WiFi...");
  display.display();

  // LED statut interne
  pinMode(LED_STATUS, OUTPUT);

  // WiFi : se connecter à l'AP créé par ESP32 #1
  WiFi.begin(ssid, password);
  int tentatives = 0;
  while (WiFi.status() != WL_CONNECTED && tentatives < 30) {
    delay(500);
    Serial.print(".");
    tentatives++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connecté !");
    Serial.print("IP : "); Serial.println(WiFi.localIP());
    display.clearDisplay();
    display.setCursor(0, 0);  display.println("WiFi OK");
    display.setCursor(0, 12); display.println(WiFi.localIP().toString());
    display.setCursor(0, 24); display.println("En attente telem...");
    display.display();
  } else {
    Serial.println("\nEchec WiFi !");
    display.clearDisplay();
    display.setCursor(0, 0); display.println("ERREUR WiFi !");
    display.display();
  }

  udp.begin(port_udp);
  Serial.println("ESP32 #2 pret.");
}

// ── LOOP ───────────────────────────────────────────────────────────────────────
void loop() {
  // ── 1) Réception paquet UDP ──────────────────────────────────────────────
  int taille = udp.parsePacket();
  if (taille > 0 && taille < (int)sizeof(buffer) - 1) {
    int lu = udp.read(buffer, sizeof(buffer) - 1);
    buffer[lu] = '\0';

    if (parseJSON(buffer)) {
      paquetsRecus++;
      dernierPaquet = millis();
      // LED interne clignote à chaque paquet reçu
      digitalWrite(LED_STATUS, (paquetsRecus % 2) ? HIGH : LOW);

      Serial.print("Telem #");
      Serial.print(paquetsRecus);
      Serial.print(" ALT=");
      Serial.print(telem.alt, 2);
      Serial.print("m MODE=");
      Serial.println(nomMode(telem.mode));
    }
  }

  // ── 2) Timeout 2 s sans paquet → invalide ───────────────────────────────
  if (telem.valide && (millis() - dernierPaquet > 2000)) {
    telem.valide = false;
    Serial.println("Timeout telem !");
  }

  // ── 3) Rafraîchissement OLED toutes les 100 ms ──────────────────────────
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 100) {
    afficherOLED();
    dernierAffichage = millis();
  }
}