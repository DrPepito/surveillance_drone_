// =============================================================================
// telemetry_sender_v3.cpp — ESP32 #1 (Station Sol / Serveur)
//
// Rôle :
//   - Reçoit télémétrie JSON depuis PC (UART Serial0) → broadcast UDP vers ESP32 #2
//   - Reçoit commandes RC depuis ESP32 #2 (UDP :3000) → relay au PC (UART)
//   - Récupère le flux JPEG de l'ESP32-CAM (HTTP client, WiFi) → relay au PC (UART)
//   - Affiche état sur OLED
//
// Flux :
//   PC ──UART──► ESP32#1 ──UDP:1234──► ESP32#2
//   PC ◄──UART── ESP32#1 ◄──UDP:3000── ESP32#2          (RC)
//   PC ◄──UART── ESP32#1 ◄──HTTP GET── ESP32-CAM :80    (images JPEG)
//
// ── PROTOCOLE UART vers PC (multiplexage) ────────────────────────────────────
// Chaque trame envoyée au PC commence par un en-tête fixe de 6 octets :
//   [0]   = 'T' (0x54) ou 'I' (0x49)   → type : Telemetrie ou Image
//   [1-4] = taille payload en uint32 little-endian
//   [5]   = '\n' (séparateur)
// Puis le payload :
//   - Type 'T' : JSON brut (texte)
//   - Type 'I' : JPEG brut (binaire, 0xFF 0xD8 ... 0xFF 0xD9)
//
// CORRECTIONS v3.3 :
//   - Route scan caméra corrigée : /capture.jpg (harmonisé avec ESP32-CAM)
//   - udp_telem.begin() retiré (port d'émission, pas de réception)
//   - Alias /capture conservé en fallback dans le scan
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>

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
const char* ip_drone     = "192.168.4.2";   // ESP32 #2 (première IP attribuée)

#define PORT_TELEM_OUT  1234   // telem → ESP32 #2
#define PORT_RC_IN      3000   // RC    ← ESP32 #2

WiFiUDP udp_telem;
WiFiUDP udp_rc;

char buf_rc[256];

// ── LED STATUS ─────────────────────────────────────────────────────────────────
#define LED_STATUS 2

// ── PROTOCOLE UART MULTIPLEXÉ ─────────────────────────────────────────────────
#define FRAME_TYPE_TELEM  'T'
#define FRAME_TYPE_IMAGE  'I'

void envoyerTrameUART(char type, const uint8_t* data, uint32_t len) {
  uint8_t header[6];
  header[0] = type;
  header[1] = (len      ) & 0xFF;
  header[2] = (len >> 8 ) & 0xFF;
  header[3] = (len >> 16) & 0xFF;
  header[4] = (len >> 24) & 0xFF;
  header[5] = '\n';

  Serial.write(header, 6);
  Serial.write(data, len);
}

// ── ÉTAT ──────────────────────────────────────────────────────────────────────
struct Telemetrie {
  float alt        = 0.0f;
  float vz         = 0.0f;
  float vx         = 0.0f;
  float vy         = 0.0f;
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

struct CmdRC {
  float roll     = 0.0f;
  float pitch    = 0.0f;
  float yaw      = 0.0f;
  float throttle = 0.0f;
  int   mode     = 0;
  bool  arm      = false;
  unsigned long ts = 0;
};

Telemetrie telem;
CmdRC      cmd;

unsigned long paquetsEnvoyes = 0;
unsigned long cmdRecues      = 0;
unsigned long framesEnvoyees = 0;
unsigned long framesErreurs  = 0;

// ── Utilitaires ───────────────────────────────────────────────────────────────
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

// ── Parse télémétrie JSON depuis PC ──────────────────────────────────────────
bool parseTelemetrie(const String& json) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

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

// ── Parse RC JSON depuis ESP32 #2, relay vers PC ─────────────────────────────
bool parseEtRelayerRC(const char* json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

  cmd.roll     = constrain((float)(doc["roll"]     | 0.0f), -1.0f, 1.0f);
  cmd.pitch    = constrain((float)(doc["pitch"]    | 0.0f), -1.0f, 1.0f);
  cmd.yaw      = constrain((float)(doc["yaw"]      | 0.0f), -1.0f, 1.0f);
  cmd.throttle = constrain((float)(doc["throttle"] | 0.0f),  0.0f, 1.0f);
  cmd.mode     = doc["mode"] | 0;
  cmd.arm      = doc["arm"]  | false;
  cmd.ts       = millis();

  envoyerTrameUART(FRAME_TYPE_TELEM, (const uint8_t*)json, strlen(json));

  return true;
}

// ── DIAGNOSTIC : stations connectées à l'AP ──────────────────────────────────
void afficherStationsAP() {
  wifi_sta_list_t liste;
  esp_wifi_ap_get_sta_list(&liste);
  Serial.printf("[AP] Stations connectees : %d\n", liste.num);
  for (int i = 0; i < liste.num; i++) {
    wifi_sta_info_t sta = liste.sta[i];
    Serial.printf("[AP]   #%d MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
      i, sta.mac[0], sta.mac[1], sta.mac[2], sta.mac[3], sta.mac[4], sta.mac[5]);
  }
}

// ── SCAN AUTOMATIQUE : trouve l'IP de la caméra sur le réseau ────────────────
WiFiClient httpClient;
HTTPClient http;
String ip_cam_actuelle = "";   // vide = pas encore trouvée

bool scannerCamera() {
  Serial.println("[SCAN] Recherche camera sur 192.168.4.2 -> .10 ...");
  for (int i = 2; i <= 10; i++) {
    String candidate = "192.168.4." + String(i);

    // On tente d'abord /capture.jpg (route principale de la CAM)
    WiFiClient c;
    HTTPClient h;
    h.begin(c, "http://" + candidate + "/capture.jpg");
    h.setTimeout(700);
    int code = h.GET();
    h.end();

    Serial.printf("[SCAN]   %s -> code=%d\n", candidate.c_str(), code);
    if (code == 200) {
      ip_cam_actuelle = candidate;
      Serial.printf("[SCAN] >>> Camera trouvee sur %s <<<\n", candidate.c_str());
      return true;
    }
  }
  Serial.println("[SCAN] Aucune camera trouvee dans la plage .2-.10");
  return false;
}

// ── Récupération d'une frame JPEG depuis l'ESP32-CAM (HTTP GET) ──────────────
void recupererEtRelayerImage() {
  if (ip_cam_actuelle == "") return;

  String url = "http://" + ip_cam_actuelle + "/capture.jpg";
  http.begin(httpClient, url);
  http.setTimeout(2000);

  int code = http.GET();
  if (code == 200) {
    int len = http.getSize();
    if (len > 0 && len < 50000) {
      WiFiClient* stream = http.getStreamPtr();
      uint8_t* buf = (uint8_t*)malloc(len);
      if (buf) {
        int lu = 0;
        unsigned long t0 = millis();
        while (lu < len && millis() - t0 < 2000) {
          if (stream->available()) {
            lu += stream->read(buf + lu, len - lu);
          }
        }
        if (lu == len) {
          envoyerTrameUART(FRAME_TYPE_IMAGE, buf, len);
          framesEnvoyees++;
        } else {
          framesErreurs++;
        }
        free(buf);
      }
    }
  } else {
    framesErreurs++;
    Serial.printf("[CAM] echec GET sur %s (code=%d) — re-scan...\n",
                   ip_cam_actuelle.c_str(), code);
    ip_cam_actuelle = "";   // on oublie l'IP, on re-scannera
  }
  http.end();
}

// ── OLED ──────────────────────────────────────────────────────────────────────
void afficherOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  char buf[32];

  if (!telem.valide) {
    display.setTextSize(1);
    display.setCursor(0, 0);  display.println("ESP32 #1 SOL");
    display.setCursor(0, 10); display.println("AP: 192.168.4.1");
    display.setCursor(0, 22); display.println("Attente Python...");
    display.display();
    return;
  }

  display.setTextSize(1);
  display.setCursor(0, 0);
  int batPx = (int)(telem.bat_pct / 100.0f * 28.0f);
  batPx = max(0, min(28, batPx));
  display.drawRect(96, 1, 30, 7, SSD1306_WHITE);
  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);
  display.fillRect(97, 2, batPx, 5, SSD1306_WHITE);
  snprintf(buf, sizeof(buf), "%-9s %3.0f%%", nomMode(telem.mode), telem.bat_pct);
  display.println(buf);

  display.setCursor(0, 10);
  display.setTextSize(2);
  snprintf(buf, sizeof(buf), "ALT%5.1fm", telem.alt);
  display.println(buf);

  display.setTextSize(1);
  display.setCursor(0, 28);
  snprintf(buf, sizeof(buf), "VZ:%+5.2f THR:%3.0f%%",
           telem.vz, telem.throttle * 100.0f);
  display.println(buf);

  display.setCursor(0, 38);
  snprintf(buf, sizeof(buf), "RC R%+4.1f P%+4.1f T%3.0f%%",
           cmd.roll, cmd.pitch, cmd.throttle * 100.0f);
  display.println(buf);

  display.setCursor(0, 50);
  snprintf(buf, sizeof(buf), "TX:%lu RX:%lu IMG:%lu",
           paquetsEnvoyes, cmdRecues, framesEnvoyees);
  display.println(buf);

  display.display();
}

// ── SETUP ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(LED_STATUS, OUTPUT);

  Serial.println("\n=== ESP32 #1 — Station Sol / Serveur ===");

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    Serial.println("Erreur init OLED");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("ESP32 #1 SOL");
  display.setCursor(0, 12); display.println("Demarrage WiFi AP...");
  display.display();

  WiFi.softAP(ssid, password);
  WiFi.softAPsetHostname("esp32-sol");
  Serial.print("AP démarré, IP : ");
  Serial.println(WiFi.softAPIP());

  // CORRECTION : udp_rc uniquement (on n'écoute pas sur PORT_TELEM_OUT)
  udp_rc.begin(PORT_RC_IN);

  display.clearDisplay();
  display.setCursor(0, 0);  display.println("WiFi AP OK");
  display.setCursor(0, 12); display.println("192.168.4.1");
  display.setCursor(0, 24); display.println("Attente Python...");
  display.display();
}

// ── LOOP ───────────────────────────────────────────────────────────────────────
void loop() {
  // ── 1) Lire télémétrie depuis PC (UART, JSON par ligne) ──────────────────
  static String ligneSerial = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      ligneSerial.trim();
      if (ligneSerial.startsWith("{")) {
        if (parseTelemetrie(ligneSerial)) {
          paquetsEnvoyes++;
          udp_telem.beginPacket(ip_drone, PORT_TELEM_OUT);
          udp_telem.print(ligneSerial);
          udp_telem.endPacket();
          digitalWrite(LED_STATUS, (paquetsEnvoyes % 2) ? HIGH : LOW);
        }
      }
      ligneSerial = "";
    } else if (c != '\r') {
      ligneSerial += c;
    }
  }

  // ── 2) Recevoir RC depuis ESP32 #2 et relay vers PC ──────────────────────
  int sz = udp_rc.parsePacket();
  if (sz > 0 && sz < (int)sizeof(buf_rc) - 1) {
    buf_rc[udp_rc.read(buf_rc, sizeof(buf_rc) - 1)] = '\0';
    if (parseEtRelayerRC(buf_rc)) {
      cmdRecues++;
    }
  }

  // ── 3) Diagnostic stations AP toutes les 2s ───────────────────────────────
  static unsigned long dernierStatAP = 0;
  if (millis() - dernierStatAP >= 2000) {
    afficherStationsAP();
    dernierStatAP = millis();
  }

  // ── 4) Scan caméra si pas encore trouvée, toutes les 3s ───────────────────
  static unsigned long dernierScan = 0;
  if (ip_cam_actuelle == "" && millis() - dernierScan >= 3000) {
    scannerCamera();
    dernierScan = millis();
  }

  // ── 5) Récupérer + relayer une image caméra toutes les 100ms (~10fps) ────
  static unsigned long dernierImg = 0;
  if (ip_cam_actuelle != "" && millis() - dernierImg >= 100) {
    recupererEtRelayerImage();
    dernierImg = millis();
  }

  // ── 6) OLED toutes les 150ms ─────────────────────────────────────────────
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 150) {
    afficherOLED();
    dernierAffichage = millis();
  }
}