// =============================================================================
// telemetry_sender.cpp — ESP32 #1 (Émetteur)
//
// Rôle : reçoit la télémétrie JSON du simulateur Python (via Serial USB),
//        l'affiche sur son OLED, et la rebroadcast en UDP WiFi vers l'ESP32 #2.
//
// Python → Serial (115200 baud, JSON lignes) → ESP32 #1 → UDP WiFi → ESP32 #2
//
// Format JSON attendu depuis Python (hil_bridge.py) :
// {"alt":1.5,"vz":-0.02,"vx":0.0,"vy":0.0,"roll":0.01,"pitch":0.02,"yaw":0.5,
//  "bat_pct":98.5,"bat_v":12.4,"m0":0.55,"m1":0.55,"m2":0.55,"m3":0.55,
//  "mode":1,"throttle":0.55,"dist":0.0}
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
const char* ssid         = "ESP32_RESEAU";
const char* password     = "12345678";
const char* ip_recepteur = "192.168.4.2";
const int   port_udp     = 1234;

WiFiUDP udp;

// ── LED STATUS (LED bleue interne uniquement) ──────────────────────────────────
#define LED_STATUS 2

// ── ÉTAT TÉLÉMÉTRIE ────────────────────────────────────────────────────────────
struct Telemetrie {
  float alt        = 0.0f;
  float vz         = 0.0f;
  float vx         = 0.0f;
  float vy         = 0.0f;
  float roll       = 0.0f;   // rad
  float pitch      = 0.0f;   // rad
  float yaw        = 0.0f;   // rad
  float bat_pct    = 100.0f;
  float bat_v      = 12.6f;
  float moteurs[4] = {0, 0, 0, 0};  // 0.0–1.0
  int   mode       = 0;
  float throttle   = 0.0f;
  float dist       = 0.0f;
  bool  valide     = false;
};

Telemetrie telem;
unsigned long dernierPaquet = 0;
unsigned long paquetsRecus  = 0;

// ── Noms de modes ──────────────────────────────────────────────────────────────
const char* nomMode(int m) {
  switch(m) {
    case 0: return "SOL";
    case 1: return "DECOLLAGE";
    case 2: return "VOL";
    case 3: return "ATTERRO";
    case 4: return "URGENCE";
    default: return "?";
  }
}

// ── Parse JSON reçu depuis Python ─────────────────────────────────────────────
bool parseJSON(const String& json) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("JSON err: "); Serial.println(err.c_str());
    return false;
  }

  telem.alt        = doc["alt"]      | 0.0f;
  telem.vz         = doc["vz"]       | 0.0f;
  telem.vx         = doc["vx"]       | 0.0f;
  telem.vy         = doc["vy"]       | 0.0f;
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

// ── OLED — affichage télémétrie ────────────────────────────────────────────────
void afficherTelemetrie() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  char buf[32];

  // ── Écran d'attente ──────────────────────────────────────────────────────
  if (!telem.valide) {
    display.setTextSize(1);
    display.setCursor(0, 0);  display.println("ESP32 #1 EMETTEUR");
    display.setCursor(0, 12); display.println("Attente Python...");
    display.setCursor(0, 24); display.print("WiFi: 192.168.4.1");
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
  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);  // bout batterie
  display.fillRect(97, 2, batPx, 5, SSD1306_WHITE);

  snprintf(buf, sizeof(buf), "%-9s %3.0f%%", nomMode(telem.mode), telem.bat_pct);
  display.println(buf);

  // ── Ligne 1 : Altitude (grand texte) ────────────────────────────────────
  display.setCursor(0, 10);
  display.setTextSize(2);
  snprintf(buf, sizeof(buf), "ALT%5.1fm", telem.alt);
  display.println(buf);

  // ── Ligne 2 : VZ + Throttle ─────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(0, 28);
  snprintf(buf, sizeof(buf), "VZ:%+5.2f THR:%3.0f%%",
           telem.vz, telem.throttle * 100.0f);
  display.println(buf);

  // ── Ligne 3 : Roll / Pitch en degrés ────────────────────────────────────
  display.setCursor(0, 38);
  snprintf(buf, sizeof(buf), "R:%+5.1f P:%+5.1f",
           telem.roll  * 57.295f,
           telem.pitch * 57.295f);
  display.println(buf);

  // ── Ligne 4 : Moteurs FL FR BL BR ────────────────────────────────────────
  // Label + pourcentage + petite barre graphique par moteur
  const char* labels[4] = {"FL", "FR", "BL", "BR"};
  int xCursor = 0;
  for (int i = 0; i < 4; i++) {
    display.setCursor(xCursor, 50);
    display.print(labels[i]);

    int pct = (int)(telem.moteurs[i] * 100.0f);
    snprintf(buf, sizeof(buf), "%3d%%", pct);
    display.setCursor(xCursor, 57);
    display.print(buf);

    // Barre verticale (4px large, max 6px haut)
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
  display.println("Demarrage...");
  display.display();

  // LED statut interne
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  // WiFi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(port_udp);

  display.clearDisplay();
  display.setCursor(0, 0);  display.println("WiFi AP OK");
  display.setCursor(0, 12); display.println("192.168.4.1");
  display.setCursor(0, 24); display.println("Attente Python...");
  display.display();

  Serial.println("ESP32 #1 pret. Attente JSON sur Serial...");
}

// ── LOOP ───────────────────────────────────────────────────────────────────────
void loop() {
  // ── 1) Lire Serial depuis Python (JSON par ligne) ────────────────────────
  static String ligneSerial = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      ligneSerial.trim();
      if (ligneSerial.length() > 2) {
        if (parseJSON(ligneSerial)) {
          paquetsRecus++;
          dernierPaquet = millis();

          // ── 2) Retransmettre en UDP WiFi vers ESP32 #2 ──────────────────
          udp.beginPacket(ip_recepteur, port_udp);
          udp.print(ligneSerial);
          udp.endPacket();

          // LED status clignote à chaque paquet reçu
          digitalWrite(LED_STATUS, (paquetsRecus % 2) ? HIGH : LOW);
        }
      }
      ligneSerial = "";
    } else {
      ligneSerial += c;
    }
  }

  // ── 3) Timeout : données obsolètes après 2 s ─────────────────────────────
  if (telem.valide && (millis() - dernierPaquet > 2000)) {
    telem.valide = false;
    Serial.println("Timeout télémétrie !");
  }

  // ── 4) Rafraîchissement OLED toutes les 100 ms ───────────────────────────
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 100) {
    afficherTelemetrie();
    dernierAffichage = millis();
  }
}