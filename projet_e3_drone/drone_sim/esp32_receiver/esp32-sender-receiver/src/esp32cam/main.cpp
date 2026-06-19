/* ============================================================
   ESP32-CAM (AI-Thinker) — Mode Station WiFi + capture JPEG
   
   CORRECTION v2 :
   - Mode STA (client) au lieu de AP : rejoint le réseau créé
     par ESP32 #1 (sender), SSID "ESP32_RESEAU".
   - Route HTTP : http://<ip_cam>/capture.jpg  (harmonisé avec sender)
   - L'ESP32 #1 peut donc scanner .2-.10 et trouver la CAM automatiquement.
   ============================================================ */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp32cam.h>

// ---------- 1. WIFI (rejoint l'AP de ESP32 #1) ----------
const char* STA_SSID = "ESP32_RESEAU";
const char* STA_PASS = "12345678";

WebServer server(80);

// ---------- 2. HANDLER CAPTURE ----------
void handleCapture() {
  auto img = esp32cam::capture();
  if (img == nullptr) {
    Serial.println("[CAM] Erreur capture frame");
    server.send(500, "text/plain", "Capture failed");
    return;
  }
  server.setContentLength(img->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  img->writeTo(client);
}

// ---------- 3. PAGE D'ACCUEIL ----------
void handleIndex() {
  String html = "<html><head><title>ESP32-CAM</title>"
                "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<style>"
                "body{text-align:center;font-family:sans-serif;background:#111;color:#fff;padding:20px}"
                "img{max-width:95%;border:2px solid #444;border-radius:6px;margin-top:10px}"
                "</style></head>"
                "<body>"
                "<h2>ESP32-CAM</h2>"
                "<p>IP: " + WiFi.localIP().toString() + "</p>"
                "<img src='/capture.jpg' />"
                "<p><a href='/capture.jpg' style='color:#0af'>Recharger l'image</a></p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

// ---------- 4. SETUP ----------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("[BOOT] Démarrage ESP32-CAM (mode STA)...");

  // ---------- Config caméra ----------
  using namespace esp32cam;

  auto res = Resolution::find(1024, 768);
  Config cfg;
  cfg.setPins(pins::AiThinker);
  cfg.setResolution(res);
  cfg.setJpeg(80);

  bool camOk = Camera.begin(cfg);
  Serial.println(camOk ? "[CAM] Caméra initialisée !"
                        : "[CAM] ÉCHEC init caméra");

  // ---------- Connexion WiFi STA ----------
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("[WIFI] Connexion à ");
  Serial.print(STA_SSID);

  int tentatives = 0;
  while (WiFi.status() != WL_CONNECTED && tentatives < 30) {
    delay(500);
    Serial.print(".");
    tentatives++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("--------------------------------------------------");
    Serial.print("[WIFI] Connecté ! IP : ");
    Serial.println(WiFi.localIP());
    Serial.print("[INFO] Accès caméra : http://");
    Serial.print(WiFi.localIP());
    Serial.println("/capture.jpg");
    Serial.println("--------------------------------------------------");
  } else {
    Serial.println("[WIFI] ÉCHEC connexion — vérifier que ESP32 #1 est démarré");
  }

  // ---------- Routes HTTP ----------
  server.on("/",           handleIndex);
  server.on("/capture.jpg", handleCapture);
  // Alias sans extension pour compatibilité éventuelle
  server.on("/capture",    handleCapture);
  server.begin();
  Serial.println("[HTTP] Serveur démarré");
}

// ---------- 5. LOOP ----------
void loop() {
  server.handleClient();
}