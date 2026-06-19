// =============================================================================
// drone_receiver_v2.cpp — ESP32 #2 (Drone)
//
// Rôle :
//   - Reçoit télémétrie depuis ESP32 #1 (UDP :1234) → affiche OLED
//   - Reçoit commandes RC depuis ESP32 #1 (UDP :2000) → stocke + prépare FC
//   - Renvoie RC vers ESP32 #1 (UDP :3000) pour relay au PC
//   - Pilote ESP32-CAM via UART2 (Serial2)
//   - Sert le flux MJPEG HTTP sur :80 (récupéré depuis CAM via UART)
//   - Interface FC : UART1 (Serial1) prêt pour Betaflight MSP ou PWM
//
// Connexions physiques :
//   ESP32 #2 GPIO16 (RX2) ←── ESP32-CAM GPIO1 (U0TXD)
//   ESP32 #2 GPIO17 (TX2) ──► ESP32-CAM GPIO3 (U0RXD)
//   ESP32 #2 GPIO32 (RX1) ←── FC UART TX  (futur)
//   ESP32 #2 GPIO33 (TX1) ──► FC UART RX  (futur)
//
// Flux réseau :
//   ESP32#1 ──UDP:1234──► ESP32#2  (telem)
//   ESP32#1 ──UDP:2000──► ESP32#2  (RC commandes)
//   ESP32#2 ──UDP:3000──► ESP32#1  (RC relay vers PC)
//   GCS     ◄──HTTP:80── ESP32#2   (flux MJPEG)
//
// CORRECTIONS v2.1 :
//   - Buffer MJPEG local (30Ko) déplacé en global → évite stack overflow
//     lors du stream HTTP (la pile de la tâche Arduino est ~8Ko)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ── OLED ──────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDR  0x3C
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── WIFI ───────────────────────────────────────────────────────────────────────
const char* ssid     = "ESP32_RESEAU";
const char* password = "12345678";

// ── PORTS UDP ──────────────────────────────────────────────────────────────────
#define PORT_TELEM_IN   1234   // telem reçue depuis ESP32 #1
#define PORT_RC_IN      2000   // RC reçues depuis ESP32 #1
#define PORT_RC_OUT     3000   // RC relayées vers ESP32 #1 (→ PC)

WiFiUDP udp_telem;
WiFiUDP udp_rc;

char buf_telem[512];
char buf_rc[256];

IPAddress ip_sol;   // IP ESP32 #1, auto-détectée

// ── UART ESP32-CAM (Serial2) ──────────────────────────────────────────────────
#define CAM_UART_RX   16
#define CAM_UART_TX   17
#define CAM_BAUD      921600

// ── UART FC — Flight Controller (Serial1, futur) ──────────────────────────────
#define FC_UART_RX    32
#define FC_UART_TX    33
#define FC_BAUD       115200

// ── LED STATUS ─────────────────────────────────────────────────────────────────
#define LED_STATUS 2

// ── SERVEUR HTTP MJPEG ────────────────────────────────────────────────────────
WebServer server(80);

// Buffer frame JPEG courante — EN GLOBAL pour éviter un stack overflow
// (30Ko sur la pile de la tâche Arduino provoquerait un crash)
#define FRAME_BUF_SIZE (30 * 1024)
uint8_t  frameBuf[FRAME_BUF_SIZE];
size_t   frameLen    = 0;
bool     frameReady  = false;
portMUX_TYPE frameMux = portMUX_INITIALIZER_UNLOCKED;

// Buffer de travail pour lireCAMUART() — aussi en global (même raison)
static uint8_t tmpBuf[FRAME_BUF_SIZE];

// Buffer de copie locale pour handleStream() — en global
uint8_t localBuf[FRAME_BUF_SIZE];

// ── ÉTAT TÉLÉMÉTRIE ────────────────────────────────────────────────────────────
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
  bool  valide     = false;
};

// ── COMMANDES RC ──────────────────────────────────────────────────────────────
struct CmdRC {
  float roll     = 0.0f;
  float pitch    = 0.0f;
  float yaw      = 0.0f;
  float throttle = 0.0f;
  int   mode     = 0;
  bool  arm      = false;
  unsigned long ts = 0;
};

// ── INTERFACE FC (MSP simplifié, futur Betaflight) ────────────────────────────
struct MSP_RC {
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint16_t throttle;
  uint16_t arm;
};

Telemetrie telem;
CmdRC      cmd;
MSP_RC     msp;

unsigned long paquetsRecus = 0;
unsigned long cmdRecues    = 0;
bool          sol_connecte = false;

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

uint16_t toMicros(float v, float vmin = -1.0f, float vmax = 1.0f,
                  uint16_t umin = 1000, uint16_t umax = 2000) {
  float norm = (v - vmin) / (vmax - vmin);
  norm = constrain(norm, 0.0f, 1.0f);
  return (uint16_t)(umin + norm * (umax - umin));
}

// ── Parse télémétrie depuis ESP32 #1 ─────────────────────────────────────────
bool parseTelemetrie(const char* json) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

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
  telem.valide     = true;
  return true;
}

// ── Parse commandes RC depuis ESP32 #1 ───────────────────────────────────────
bool parseRC(const char* json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return false;

  cmd.roll     = constrain((float)(doc["roll"]     | 0.0f), -1.0f, 1.0f);
  cmd.pitch    = constrain((float)(doc["pitch"]    | 0.0f), -1.0f, 1.0f);
  cmd.yaw      = constrain((float)(doc["yaw"]      | 0.0f), -1.0f, 1.0f);
  cmd.throttle = constrain((float)(doc["throttle"] | 0.0f),  0.0f, 1.0f);
  cmd.mode     = doc["mode"] | 0;
  cmd.arm      = doc["arm"]  | false;
  cmd.ts       = millis();

  msp.roll     = toMicros(cmd.roll);
  msp.pitch    = toMicros(cmd.pitch);
  msp.yaw      = toMicros(cmd.yaw);
  msp.throttle = toMicros(cmd.throttle, 0.0f, 1.0f);
  msp.arm      = cmd.arm ? 2000 : 1000;

  return true;
}

// ── Envoi RC vers FC via MSP (Serial1) ───────────────────────────────────────
void sendToFC() {
  if (!cmd.arm) return;

  if (millis() - cmd.ts > 500) {
    msp.throttle = 1000;
    msp.roll     = 1500;
    msp.pitch    = 1500;
    msp.yaw      = 1500;
    msp.arm      = 1000;
  }

  uint8_t  payload[16];
  uint16_t channels[8] = {
    msp.roll, msp.pitch, msp.yaw, msp.throttle,
    msp.arm, 1000, 1000, 1000
  };
  for (int i = 0; i < 8; i++) {
    payload[i*2]   = channels[i] & 0xFF;
    payload[i*2+1] = (channels[i] >> 8) & 0xFF;
  }

  uint8_t checksum = 16 ^ 200;
  for (int i = 0; i < 16; i++) checksum ^= payload[i];

  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');
  Serial1.write((uint8_t)16);
  Serial1.write((uint8_t)200);
  Serial1.write(payload, 16);
  Serial1.write(checksum);
}

// ── Relay RC vers ESP32 #1 ───────────────────────────────────────────────────
void relayerRCVersSol() {
  if (!sol_connecte) return;

  StaticJsonDocument<256> doc;
  doc["roll"]     = cmd.roll;
  doc["pitch"]    = cmd.pitch;
  doc["yaw"]      = cmd.yaw;
  doc["throttle"] = cmd.throttle;
  doc["mode"]     = cmd.mode;
  doc["arm"]      = cmd.arm;

  char out[256];
  serializeJson(doc, out);

  udp_rc.beginPacket(ip_sol, PORT_RC_OUT);
  udp_rc.write((uint8_t*)out, strlen(out));
  udp_rc.endPacket();
}

// ── Lecture frame JPEG depuis ESP32-CAM (UART2) ───────────────────────────────
void lireCAMUART() {
  static size_t  tmpLen   = 0;
  static bool    inFrame  = false;
  static uint8_t prevByte = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (!inFrame) {
      if (prevByte == 0xFF && b == 0xD8) {
        inFrame = true;
        tmpLen = 0;
        tmpBuf[tmpLen++] = 0xFF;
        tmpBuf[tmpLen++] = 0xD8;
      }
    } else {
      if (tmpLen < FRAME_BUF_SIZE) {
        tmpBuf[tmpLen++] = b;
      }

      if (prevByte == 0xFF && b == 0xD9) {
        portENTER_CRITICAL(&frameMux);
        memcpy(frameBuf, tmpBuf, tmpLen);
        frameLen   = tmpLen;
        frameReady = true;
        portEXIT_CRITICAL(&frameMux);

        inFrame = false;
        tmpLen  = 0;
      }
    }
    prevByte = b;
  }
}

// ── Handler HTTP : page index ────────────────────────────────────────────────
void handleIndex() {
  String html =
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<title>Drone FPV</title>"
    "<style>body{background:#111;margin:0;font-family:sans-serif;color:#eee}"
    "h2{text-align:center;padding:10px;margin:0;font-size:16px}"
    "img{display:block;margin:auto;max-width:100%;border:1px solid #333}"
    ".info{text-align:center;font-size:12px;color:#888;padding:4px}"
    "</style></head><body>"
    "<h2>ESP32 Drone — FPV Feed</h2>"
    "<img src='/stream'/>"
    "<div class='info'>Stream MJPEG via ESP32 #2</div>"
    "</body></html>";
  server.send(200, "text/html", html);
}

// ── Handler HTTP : stream MJPEG ──────────────────────────────────────────────
void handleStream() {
  WiFiClient client = server.client();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Cache-Control: no-cache");
  client.println("Connection: close");
  client.println();

  Serial.println("Client stream connecté");

  while (client.connected()) {
    if (!frameReady) {
      lireCAMUART();
      delay(1);
      continue;
    }

    // Copie sous mutex dans le buffer global (pas sur la pile)
    size_t localLen = 0;
    portENTER_CRITICAL(&frameMux);
    localLen   = frameLen;
    memcpy(localBuf, frameBuf, frameLen);
    frameReady = false;
    portEXIT_CRITICAL(&frameMux);

    if (localLen == 0) continue;

    client.printf("--frame\r\n");
    client.printf("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", localLen);
    client.write(localBuf, localLen);
    client.printf("\r\n");
  }

  Serial.println("Client stream déconnecté");
}

// ── OLED ──────────────────────────────────────────────────────────────────────
void afficherOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  char buf[32];

  if (!telem.valide && !sol_connecte) {
    display.setTextSize(1);
    display.setCursor(0, 0);  display.println("ESP32 #2 DRONE");
    display.setCursor(0, 10); display.println(WiFi.localIP().toString());
    display.setCursor(0, 22); display.println("Attente sol...");
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
  snprintf(buf, sizeof(buf), "%-8s %3.0f%%", nomMode(telem.mode), telem.bat_pct);
  display.println(buf);

  display.setCursor(0, 10);
  display.setTextSize(2);
  snprintf(buf, sizeof(buf), "%5.1fm", telem.alt);
  display.println(buf);

  display.setTextSize(1);
  display.setCursor(0, 28);
  snprintf(buf, sizeof(buf), "R%+4.1f P%+4.1f T%3.0f%%",
           cmd.roll, cmd.pitch, cmd.throttle * 100.0f);
  display.println(buf);

  display.setCursor(0, 38);
  snprintf(buf, sizeof(buf), "FC R%4d T%4d %s",
           msp.roll, msp.throttle, cmd.arm ? "ARM" : "DIS");
  display.println(buf);

  display.setCursor(0, 50);
  snprintf(buf, sizeof(buf), "SOL:%s CAM:%s #%lu",
           sol_connecte ? "OK" : "--",
           frameReady   ? "OK" : "--",
           cmdRecues);
  display.println(buf);

  display.display();
}

// ── SETUP ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(FC_BAUD,  SERIAL_8N1, FC_UART_RX,  FC_UART_TX);
  Serial2.begin(CAM_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);

  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(LED_STATUS, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDR)) {
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("ESP32 #2 DRONE");
  display.setCursor(0, 12); display.println("Connexion WiFi...");
  display.display();

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  int tentatives = 0;
  while (WiFi.status() != WL_CONNECTED && tentatives < 30) {
    delay(500);
    Serial.print(".");
    tentatives++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    ip_sol = WiFi.gatewayIP();
    sol_connecte = true;
    Serial.println("\nWiFi OK : " + WiFi.localIP().toString());
    Serial.println("Sol IP  : " + ip_sol.toString());

    display.clearDisplay();
    display.setCursor(0, 0);  display.println("WiFi OK");
    display.setCursor(0, 10); display.println(WiFi.localIP().toString());
    display.setCursor(0, 22); display.print("Sol: ");
    display.println(ip_sol.toString());
    display.display();
  } else {
    Serial.println("\nEchec WiFi !");
  }

  udp_telem.begin(PORT_TELEM_IN);
  udp_rc.begin(PORT_RC_IN);

  server.on("/",       handleIndex);
  server.on("/stream", handleStream);
  server.begin();
  Serial.println("Serveur HTTP démarré sur :80");
  Serial.println("Stream : http://" + WiFi.localIP().toString() + "/stream");
  Serial.println("ESP32 #2 prêt.");
}

// ── LOOP ───────────────────────────────────────────────────────────────────────
void loop() {
  // ── 1) Lire UART CAM en continu ──────────────────────────────────────────
  lireCAMUART();

  // ── 2) Serveur HTTP ───────────────────────────────────────────────────────
  server.handleClient();

  // ── 3) Réception télémétrie depuis ESP32 #1 ──────────────────────────────
  int sz_telem = udp_telem.parsePacket();
  if (sz_telem > 0 && sz_telem < (int)sizeof(buf_telem) - 1) {
    buf_telem[udp_telem.read(buf_telem, sizeof(buf_telem) - 1)] = '\0';
    if (parseTelemetrie(buf_telem)) {
      paquetsRecus++;
      digitalWrite(LED_STATUS, (paquetsRecus % 2) ? HIGH : LOW);
    }
  }

  // ── 4) Réception RC depuis ESP32 #1 ──────────────────────────────────────
  int sz_rc = udp_rc.parsePacket();
  if (sz_rc > 0 && sz_rc < (int)sizeof(buf_rc) - 1) {
    buf_rc[udp_rc.read(buf_rc, sizeof(buf_rc) - 1)] = '\0';
    if (parseRC(buf_rc)) {
      cmdRecues++;
      relayerRCVersSol();
    }
  }

  // ── 5) Envoi RC vers FC toutes les 20ms (50Hz) ───────────────────────────
  static unsigned long dernierFC = 0;
  if (millis() - dernierFC >= 20) {
    sendToFC();
    dernierFC = millis();
  }

  // ── 6) Timeout télémétrie 2s ─────────────────────────────────────────────
  static unsigned long dernierTelem = 0;
  if (telem.valide) dernierTelem = millis();
  if (millis() - dernierTelem > 2000) telem.valide = false;

  // ── 7) OLED toutes les 100ms ─────────────────────────────────────────────
  static unsigned long dernierOLED = 0;
  if (millis() - dernierOLED >= 100) {
    afficherOLED();
    dernierOLED = millis();
  }
}