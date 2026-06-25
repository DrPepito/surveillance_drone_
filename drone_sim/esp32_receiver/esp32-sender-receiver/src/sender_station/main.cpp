// =============================================================================
// telemetry_sender_v4.cpp — ESP32 #0 (Station Sol / Serveur)
//
// Rôle :
//   - Reçoit JSON depuis PC (UART Serial0) contenant TÉLÉMÉTRIE + RC mélangées
//   - Sépare les deux flux et les envoie sur des ports UDP distincts :
//       → Port 1234 : JSON télémétrie pur        → ESP32 #1 (drone)
//       → Port 2000 : JSON commandes RC pur       → ESP32 #1 (drone)
//   - Reçoit commandes RC en écho depuis ESP32 #1 (UDP :3000) → relay au PC
//   - Récupère le flux JPEG de l'ESP32-CAM (HTTP) → relay au PC (UART)
//   - Affiche état sur OLED
//
// Flux :
//   PC ──UART──► ESP32#0 ──UDP:1234──► ESP32#1  (télémétrie)
//   PC ──UART──► ESP32#0 ──UDP:2000──► ESP32#1  (RC)         ← NOUVEAU
//   PC ◄──UART── ESP32#0 ◄──UDP:3000── ESP32#1  (écho RC)
//   PC ◄──UART── ESP32#0 ◄──HTTP GET── ESP32-CAM :80
//
// Pourquoi deux ports UDP ?
//   drone_receiver_v2.cpp écoute la télémétrie sur le port 1234
//   et les RC sur le port 2000. En fusionnant les deux dans un seul
//   paquet, le drone reçoit des champs inutiles et doit parser deux fois.
//   Séparer les ports permet à chaque flux de garder sa propre cadence
//   et son propre watchdog côté drone.
//
// Format JSON RC envoyé sur port 2000 :
//   {"roll":0.0,"pitch":0.0,"yaw":0.0,"throttle":0.0,"arm":false,"mode":0}
//
// Format JSON télémétrie envoyé sur port 1234 (inchangé) :
//   {"alt":...,"vz":...,"vx":...,"vy":...,"roll":...,"pitch":...,"yaw":...
//    "bat_pct":...,"bat_v":...,"m0"..,"m3":...,"mode":...,"throttle":...,"dist":...}
//
// PROTOCOLE UART vers PC (multiplexage — inchangé) :
//   [type(1)] [taille uint32 LE (4)] ['\n'(1)] [payload]
//   type 'T' = JSON RC reçu en écho
//   type 'I' = JPEG caméra
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
const char* ssid      = "ESP32_RESEAU";
const char* password  = "12345678";
const char* ip_drone  = "192.168.4.2";   // ESP32 #1 (première IP client)

#define PORT_TELEM_OUT  1234   // télémétrie  → ESP32 #1
#define PORT_RC_OUT     2000   // commandes RC → ESP32 #1  ← NOUVEAU port dédié
#define PORT_RC_IN      3000   // écho RC     ← ESP32 #1

WiFiUDP udp_telem;   // émission télémétrie (pas de begin() nécessaire)
WiFiUDP udp_rc_out;  // émission RC vers drone
WiFiUDP udp_rc_in;   // réception écho RC depuis drone

char buf_rc[256];

// ── LED ────────────────────────────────────────────────────────────────────────
#define LED_STATUS 2

// ── PROTOCOLE UART MULTIPLEXÉ ─────────────────────────────────────────────────
#define FRAME_TYPE_TELEM  'T'
#define FRAME_TYPE_IMAGE  'I'

void envoyerTrameUART(char type, const uint8_t* data, uint32_t len) {
  uint8_t header[6];
  header[0] = (uint8_t)type;
  header[1] = (len      ) & 0xFF;
  header[2] = (len >>  8) & 0xFF;
  header[3] = (len >> 16) & 0xFF;
  header[4] = (len >> 24) & 0xFF;
  header[5] = '\n';
  Serial.write(header, 6);
  Serial.write(data, len);
}

// ── ÉTAT LOCAL ────────────────────────────────────────────────────────────────
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
  bool  valide   = false;
};

Telemetrie telem;
CmdRC      cmd;

unsigned long paquetsTelemEnvoyes = 0;
unsigned long paquetsRcEnvoyes    = 0;
unsigned long cmdRecues           = 0;
unsigned long framesEnvoyees      = 0;
unsigned long framesErreurs       = 0;

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

// ── Parse le JSON entrant depuis le PC ───────────────────────────────────────
// Le PC envoie UN SEUL JSON contenant à la fois télémétrie et RC.
// On extrait les deux blocs séparément.
bool parseJsonPC(const String& json) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

  // --- Télémétrie ---
  telem.alt        = doc["alt"]      | telem.alt;
  telem.vz         = doc["vz"]       | telem.vz;
  telem.vx         = doc["vx"]       | telem.vx;
  telem.vy         = doc["vy"]       | telem.vy;
  telem.roll       = doc["roll"]     | telem.roll;
  telem.pitch      = doc["pitch"]    | telem.pitch;
  telem.yaw        = doc["yaw"]      | telem.yaw;
  telem.bat_pct    = doc["bat_pct"]  | telem.bat_pct;
  telem.bat_v      = doc["bat_v"]    | telem.bat_v;
  telem.moteurs[0] = doc["m0"]       | telem.moteurs[0];
  telem.moteurs[1] = doc["m1"]       | telem.moteurs[1];
  telem.moteurs[2] = doc["m2"]       | telem.moteurs[2];
  telem.moteurs[3] = doc["m3"]       | telem.moteurs[3];
  telem.mode       = doc["mode"]     | telem.mode;
  telem.throttle   = doc["throttle"] | telem.throttle;
  telem.dist       = doc["dist"]     | telem.dist;
  telem.valide     = true;

  // --- Commandes RC (champs cmd_* envoyés par hil_bridge_serial.py) ---
  // On accepte aussi les champs sans préfixe pour compatibilité.
  bool rcPresent = false;

  if (doc.containsKey("cmd_roll")) {
    cmd.roll  = constrain((float)doc["cmd_roll"],  -1.0f, 1.0f);
    rcPresent = true;
  } else if (doc.containsKey("roll_cmd")) {
    cmd.roll  = constrain((float)doc["roll_cmd"],  -1.0f, 1.0f);
    rcPresent = true;
  }

  if (doc.containsKey("cmd_pitch")) {
    cmd.pitch = constrain((float)doc["cmd_pitch"], -1.0f, 1.0f);
    rcPresent = true;
  }

  if (doc.containsKey("cmd_yaw")) {
    cmd.yaw   = constrain((float)doc["cmd_yaw"],   -1.0f, 1.0f);
    rcPresent = true;
  }

  if (doc.containsKey("cmd_thr")) {
    cmd.throttle = constrain((float)doc["cmd_thr"], 0.0f, 1.0f);
    rcPresent    = true;
  }

  if (doc.containsKey("arm")) {
    cmd.arm   = (bool)doc["arm"];
    rcPresent = true;
  }

  if (doc.containsKey("mode")) {
    cmd.mode = (int)doc["mode"];
  }

  if (rcPresent) {
    cmd.ts     = millis();
    cmd.valide = true;
  }

  return true;
}

// ── Construit et envoie le paquet télémétrie (port 1234) ─────────────────────
void envoyerTelemetrie() {
  StaticJsonDocument<384> doc;
  doc["alt"]      = telem.alt;
  doc["vz"]       = telem.vz;
  doc["vx"]       = telem.vx;
  doc["vy"]       = telem.vy;
  doc["roll"]     = telem.roll;
  doc["pitch"]    = telem.pitch;
  doc["yaw"]      = telem.yaw;
  doc["bat_pct"]  = telem.bat_pct;
  doc["bat_v"]    = telem.bat_v;
  doc["m0"]       = telem.moteurs[0];
  doc["m1"]       = telem.moteurs[1];
  doc["m2"]       = telem.moteurs[2];
  doc["m3"]       = telem.moteurs[3];
  doc["mode"]     = telem.mode;
  doc["throttle"] = telem.throttle;
  doc["dist"]     = telem.dist;

  char buf[384];
  size_t len = serializeJson(doc, buf);

  udp_telem.beginPacket(ip_drone, PORT_TELEM_OUT);
  udp_telem.write((const uint8_t*)buf, len);
  udp_telem.endPacket();

  paquetsTelemEnvoyes++;
  digitalWrite(LED_STATUS, (paquetsTelemEnvoyes % 2) ? HIGH : LOW);
}

// ── Construit et envoie le paquet RC (port 2000) ─────────────────────────────
// Paquet minimal : uniquement les axes + arm. Le drone n'a pas besoin
// de toute la télémétrie pour calculer les µs MSP.
void envoyerRC() {
  StaticJsonDocument<128> doc;
  doc["roll"]     = cmd.roll;
  doc["pitch"]    = cmd.pitch;
  doc["yaw"]      = cmd.yaw;
  doc["throttle"] = cmd.throttle;
  doc["arm"]      = cmd.arm;
  doc["mode"]     = cmd.mode;

  char buf[128];
  size_t len = serializeJson(doc, buf);

  udp_rc_out.beginPacket(ip_drone, PORT_RC_OUT);
  udp_rc_out.write((const uint8_t*)buf, len);
  udp_rc_out.endPacket();

  paquetsRcEnvoyes++;
}

// ── Reçoit l'écho RC depuis ESP32 #1 et le relay au PC (UART) ────────────────
bool parseEtRelayerEchoRC(const char* json) {
  // Validation minimale : doit ressembler à du JSON
  if (json[0] != '{') return false;

  // On relay tel quel vers le PC sous forme de trame 'T'
  envoyerTrameUART(FRAME_TYPE_TELEM, (const uint8_t*)json, strlen(json));
  cmdRecues++;
  return true;
}

// ── Diagnostic AP ─────────────────────────────────────────────────────────────
void afficherStationsAP() {
  wifi_sta_list_t liste;
  esp_wifi_ap_get_sta_list(&liste);
  Serial.printf("[AP] Stations connectees : %d\n", liste.num);
  for (int i = 0; i < liste.num; i++) {
    wifi_sta_info_t sta = liste.sta[i];
    Serial.printf("[AP]   #%d MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
      i, sta.mac[0], sta.mac[1], sta.mac[2],
         sta.mac[3], sta.mac[4], sta.mac[5]);
  }
}

// ── Scan caméra ───────────────────────────────────────────────────────────────
WiFiClient httpClient;
HTTPClient http;
String ip_cam_actuelle = "";

bool scannerCamera() {
  Serial.println("[SCAN] Recherche camera sur 192.168.4.2 -> .10 ...");
  for (int i = 2; i <= 10; i++) {
    String candidate = "192.168.4." + String(i);
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

// ── Récupération image JPEG et relay UART ────────────────────────────────────
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
    ip_cam_actuelle = "";
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
    display.setCursor(0, 0);  display.println("ESP32 #0 SOL v4");
    display.setCursor(0, 10); display.println("AP: 192.168.4.1");
    display.setCursor(0, 22); display.println("Attente Python...");
    display.display();
    return;
  }

  // Ligne 0 : mode + batterie
  display.setTextSize(1);
  display.setCursor(0, 0);
  int batPx = (int)(telem.bat_pct / 100.0f * 28.0f);
  batPx = max(0, min(28, batPx));
  display.drawRect(96, 1, 30, 7, SSD1306_WHITE);
  display.drawRect(126, 2, 2, 4, SSD1306_WHITE);
  display.fillRect(97, 2, batPx, 5, SSD1306_WHITE);
  snprintf(buf, sizeof(buf), "%-9s %3.0f%%", nomMode(telem.mode), telem.bat_pct);
  display.println(buf);

  // Ligne 1 : altitude (grand)
  display.setTextSize(2);
  display.setCursor(0, 10);
  snprintf(buf, sizeof(buf), "ALT%5.1fm", telem.alt);
  display.println(buf);

  // Ligne 2 : VZ + throttle télémétrie
  display.setTextSize(1);
  display.setCursor(0, 28);
  snprintf(buf, sizeof(buf), "VZ:%+5.2f THR:%3.0f%%",
           telem.vz, telem.throttle * 100.0f);
  display.println(buf);

  // Ligne 3 : commandes RC envoyées au drone
  display.setCursor(0, 38);
  if (cmd.valide && millis() - cmd.ts < 2000) {
    snprintf(buf, sizeof(buf), "RC R%+4.1f T%3.0f%% %s",
             cmd.roll, cmd.throttle * 100.0f, cmd.arm ? "ARM" : "DIS");
  } else {
    snprintf(buf, sizeof(buf), "RC ---  ATTENTE");
  }
  display.println(buf);

  // Ligne 4 : compteurs
  display.setCursor(0, 50);
  snprintf(buf, sizeof(buf), "TL:%lu RC:%lu IMG:%lu",
           paquetsTelemEnvoyes, paquetsRcEnvoyes, framesEnvoyees);
  display.println(buf);

  display.display();
}

// ── SETUP ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(LED_STATUS, OUTPUT);

  Serial.println("\n=== ESP32 #0 — Station Sol v4 ===");

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    Serial.println("Erreur init OLED");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("ESP32 #0 SOL v4");
  display.setCursor(0, 12); display.println("Demarrage WiFi AP...");
  display.display();

  WiFi.softAP(ssid, password);
  WiFi.softAPsetHostname("esp32-sol");
  Serial.print("AP démarré, IP : ");
  Serial.println(WiFi.softAPIP());

  // Réception écho RC uniquement (on n'appelle pas begin() sur les UDP d'émission)
  udp_rc_in.begin(PORT_RC_IN);

  display.clearDisplay();
  display.setCursor(0, 0);  display.println("WiFi AP OK");
  display.setCursor(0, 12); display.println("192.168.4.1");
  display.setCursor(0, 24); display.println("Attente Python...");
  display.display();

  Serial.println("[OK] Ports UDP :");
  Serial.printf("     Telem OUT → drone port %d\n", PORT_TELEM_OUT);
  Serial.printf("     RC    OUT → drone port %d\n", PORT_RC_OUT);
  Serial.printf("     RC    IN  ← drone port %d\n", PORT_RC_IN);
}

// ── LOOP ───────────────────────────────────────────────────────────────────────
void loop() {

  // ── 1) Lire JSON depuis PC (UART, une ligne = une trame) ─────────────────
  static String ligneSerial = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      ligneSerial.trim();
      if (ligneSerial.startsWith("{")) {
        if (parseJsonPC(ligneSerial)) {
          // Toujours envoyer la télémétrie
          envoyerTelemetrie();

          // Envoyer les RC seulement si elles sont fraîches (présentes dans ce JSON)
          // On détecte "fraîches" = ts mis à jour dans parseJsonPC
          if (cmd.valide && millis() - cmd.ts < 100) {
            envoyerRC();
          }
        }
      }
      ligneSerial = "";
    } else if (c != '\r') {
      ligneSerial += c;
    }
  }

  // ── 2) Recevoir écho RC depuis ESP32 #1 et relay vers PC ─────────────────
  int sz = udp_rc_in.parsePacket();
  if (sz > 0 && sz < (int)sizeof(buf_rc) - 1) {
    int n = udp_rc_in.read(buf_rc, sizeof(buf_rc) - 1);
    buf_rc[n] = '\0';
    parseEtRelayerEchoRC(buf_rc);
  }

  // ── 3) Diagnostic stations AP toutes les 2s ───────────────────────────────
  static unsigned long dernierStatAP = 0;
  if (millis() - dernierStatAP >= 2000) {
    afficherStationsAP();
    dernierStatAP = millis();
  }

  // ── 4) Scan caméra si pas encore trouvée, toutes les 3s ──────────────────
  static unsigned long dernierScan = 0;
  if (ip_cam_actuelle == "" && millis() - dernierScan >= 3000) {
    scannerCamera();
    dernierScan = millis();
  }

  // ── 5) Récupérer + relayer image caméra toutes les 100ms (~10fps) ────────
  static unsigned long dernierImg = 0;
  if (ip_cam_actuelle != "" && millis() - dernierImg >= 100) {
    recupererEtRelayerImage();
    dernierImg = millis();
  }

  // ── 6) OLED toutes les 150ms ────────────────────────────────────────
  static unsigned long dernierAffichage = 0;
  if (millis() - dernierAffichage >= 150) {
    afficherOLED();
    dernierAffichage = millis();
  }
}