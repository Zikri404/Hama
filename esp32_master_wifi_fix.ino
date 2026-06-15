/*
 * ============================================================
 *  ESP32 MASTER — Dashboard Web 2 Kamera + Status Sistem
 *  Politeknik Manufaktur Bandung | Teknik Otomasi Mekatronika
 * ============================================================
 *
 *  FUNGSI:
 *  - Terima log dari CAM#1 & CAM#2 via HardwareSerial1 (GPIO4 RX, GPIO2 TX)
 *  - Serve dashboard web di port 80
 *  - Dashboard embed stream kedua kamera + status real-time
 *  - Kontrol buzzer & LED sesuai status AMAN/BAHAYA
 *
 *  AKSES DASHBOARD:
 *  1. Connect HP/Laptop ke WiFi "ESP32-CAM-System"
 *  2. Buka browser → http://192.168.4.1
 *
 *  UBAH: WIFI_SSID, WIFI_PASS, IP_CAM1, IP_CAM2
 *  IP kamera didapat dari Serial Monitor CAM saat boot
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>

// ─────────────────────────────────────────
//  KONFIGURASI HOTSPOT
// ─────────────────────────────────────────
#define AP_SSID   "ESP32-CAM-System"
#define AP_PASS   "12345678"      // Min 8 karakter, kosongkan untuk open
#define AP_IP     "192.168.4.1"  // IP master (default ESP32 AP)

// IP kamera — otomatis dari DHCP hotspot
// CAM#1 biasanya dapat 192.168.4.2
// CAM#2 biasanya dapat 192.168.4.3
String IP_CAM1 = "192.168.4.2";
String IP_CAM2 = "192.168.4.3";

// ─────────────────────────────────────────
//  KONFIGURASI
// ─────────────────────────────────────────
#define RX_FROM_CAM   4    // GPIO4 — sambung CAM UOT ke sini
#define TX_TO_CAM     2    // GPIO2 — tidak dipakai
#define DELTA_BAHAYA      10
#define TIMEOUT_KAMERA  120000

#define PIN_BUZZER    25
#define PIN_LED_MERAH 26
#define PIN_LED_HIJAU 27

// ─────────────────────────────────────────
//  OBJEK
// ─────────────────────────────────────────
WebServer server(80);

// ─────────────────────────────────────────
//  DATA KAMERA
// ─────────────────────────────────────────
struct KameraData {
  int      id;
  int      countHama;
  int      deltaHama;
  String   status;
  String   ip;
  uint32_t lastUpdate;
  bool     online;
};

KameraData kamera[2] = {
  {1, 0, 0, "BOOT", "--", 0, false},
  {2, 0, 0, "BOOT", "--", 0, false}
};

// ─────────────────────────────────────────
//  STATUS SISTEM
// ─────────────────────────────────────────
enum StatusSistem { AMAN, BAHAYA };
StatusSistem statusSistem     = AMAN;
StatusSistem statusSebelumnya = AMAN;

String serialBuffer     = "";
uint32_t lastDisplay    = 0;
uint32_t lastBuzzerTick = 0;
bool     buzzerState    = false;

// ─────────────────────────────────────────
//  PARSE LOG DARI KAMERA
// ─────────────────────────────────────────
void parseLog(String raw) {
  int startLog = raw.indexOf("##LOG##");
  int endLog   = raw.indexOf("##END##");
  if (startLog == -1 || endLog == -1) return;

  String json = raw.substring(startLog + 7, endLog);
  json.trim();

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return;

  int    camId = doc["cam"]    | 0;
  int    count = doc["count"]  | 0;
  int    delta = doc["delta"]  | 0;
  String stat  = doc["status"] | "UNKNOWN";
  String ip    = doc["ip"]     | "--";

  if (camId < 1 || camId > 2) return;
  int idx = camId - 1;

  kamera[idx].countHama  = count;
  kamera[idx].deltaHama  = delta;
  kamera[idx].status     = stat;
  kamera[idx].lastUpdate = millis();
  kamera[idx].online     = true;

  // Update IP kamera jika baru dapat
  if (ip != "--") {
    kamera[idx].ip = ip;
    if (camId == 1) IP_CAM1 = ip;
    if (camId == 2) IP_CAM2 = ip;
  }

  Serial.printf("[MASTER] ← CAM#%d: %d hama | Δ+%d | %s | IP:%s\n",
    camId, count, delta, stat.c_str(), ip.c_str());
}

// ─────────────────────────────────────────
//  STATUS SISTEM
// ─────────────────────────────────────────
StatusSistem tentukanStatus() {
  int maxDelta = max(kamera[0].deltaHama, kamera[1].deltaHama);
  return (maxDelta >= DELTA_BAHAYA) ? BAHAYA : AMAN;
}

// ─────────────────────────────────────────
//  HTML DASHBOARD
// ─────────────────────────────────────────
String buildDashboard() {
  String statusStr   = (statusSistem == BAHAYA) ? "BAHAYA" : "AMAN";
  String statusColor = (statusSistem == BAHAYA) ? "#E24B4A" : "#1D9E75";
  String statusEmoji = (statusSistem == BAHAYA) ? "🔴" : "🟢";
  String cam1Online  = kamera[0].online ? "ONLINE"  : "OFFLINE";
  String cam2Online  = kamera[1].online ? "ONLINE"  : "OFFLINE";
  String cam1Color   = kamera[0].online ? "#1D9E75" : "#E24B4A";
  String cam2Color   = kamera[1].online ? "#1D9E75" : "#E24B4A";

  // Tentukan kamera mana yang trigger bahaya
  String pesanBahaya = "";
  if (statusSistem == BAHAYA) {
    int camIdx = (kamera[0].deltaHama >= kamera[1].deltaHama) ? 0 : 1;
    pesanBahaya = "HAMA MASIF di CAM#" + String(camIdx+1) +
                  " (+" + String(kamera[camIdx].deltaHama) + " hama) — SIAPKAN PENYEMPROTAN!";
  }

  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="refresh" content="10">
<title>Sistem Deteksi Hama — Polman Bandung</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0a1628;
    color: #c8dcd0;
    font-family: 'Courier New', monospace;
    min-height: 100vh;
  }
  .header {
    background: #0d2035;
    border-bottom: 2px solid #1D9E75;
    padding: 14px 24px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .header h1 { font-size: 15px; color: #9FE1CB; letter-spacing: 0.05em; }
  .header .time { font-size: 12px; color: #5DCAA5; }
  .status-bar {
    padding: 12px 24px;
    background: )rawhtml" + statusColor + R"rawhtml(22;
    border-bottom: 1px solid )rawhtml" + statusColor + R"rawhtml(55;
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .status-badge {
    font-size: 18px;
    font-weight: bold;
    color: )rawhtml" + statusColor + R"rawhtml(;
    letter-spacing: 0.1em;
  }
  .status-msg {
    font-size: 12px;
    color: )rawhtml" + statusColor + R"rawhtml(;
    opacity: 0.85;
  }
  .container { padding: 20px 24px; }
  .cameras {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
    margin-bottom: 20px;
  }
  .cam-card {
    background: #0d2035;
    border: 1px solid #1D9E7544;
    border-radius: 8px;
    overflow: hidden;
  }
  .cam-header {
    padding: 10px 14px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: 1px solid #1D9E7530;
  }
  .cam-title { font-size: 13px; font-weight: bold; color: #9FE1CB; }
  .cam-badge {
    font-size: 10px;
    padding: 2px 8px;
    border-radius: 10px;
    font-weight: bold;
  }
  .cam-stream {
    width: 100%;
    aspect-ratio: 4/3;
    background: #060f1e;
    display: flex;
    align-items: center;
    justify-content: center;
    position: relative;
    overflow: hidden;
  }
  .cam-stream img {
    width: 100%;
    height: 100%;
    object-fit: cover;
  }
  .cam-stream .offline-msg {
    color: #E24B4A55;
    font-size: 12px;
    text-align: center;
  }
  .cam-stats {
    padding: 10px 14px;
    display: flex;
    gap: 16px;
    border-top: 1px solid #1D9E7520;
  }
  .stat { display: flex; flex-direction: column; gap: 2px; }
  .stat-label { font-size: 9px; color: #5DCAA5; text-transform: uppercase; letter-spacing: 0.08em; }
  .stat-value { font-size: 14px; font-weight: bold; color: #9FE1CB; }
  .info-grid {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 12px;
    margin-bottom: 16px;
  }
  .info-card {
    background: #0d2035;
    border: 1px solid #1D9E7530;
    border-radius: 8px;
    padding: 12px 16px;
  }
  .info-label { font-size: 9px; color: #5DCAA5; text-transform: uppercase; letter-spacing: 0.08em; margin-bottom: 4px; }
  .info-value { font-size: 13px; color: #c8dcd0; font-weight: 500; }
  .footer {
    text-align: center;
    padding: 12px;
    font-size: 10px;
    color: #1D9E7555;
    border-top: 1px solid #1D9E7520;
    margin-top: 8px;
  }
  @media (max-width: 600px) {
    .cameras { grid-template-columns: 1fr; }
    .info-grid { grid-template-columns: 1fr 1fr; }
  }
</style>
</head>
<body>

<div class="header">
  <h1>🌿 SISTEM DETEKSI HAMA — POLMAN BANDUNG</h1>
  <span class="time">Auto-refresh: 10s</span>
</div>

<div class="status-bar">
  <span class="status-badge">)rawhtml" + statusEmoji + " " + statusStr + R"rawhtml(</span>
  <span class="status-msg">)rawhtml";

  if (statusSistem == AMAN) {
    html += "LAHAN AMAN — Tidak ada lonjakan hama signifikan";
  } else {
    html += pesanBahaya;
  }

  html += R"rawhtml(</span>
</div>

<div class="container">

  <div class="cameras">
    <!-- CAM #1 -->
    <div class="cam-card">
      <div class="cam-header">
        <span class="cam-title">📷 Kamera #1 — Yellow Trap Kiri</span>
        <span class="cam-badge" style="background:)rawhtml" + cam1Color + R"rawhtml(22;color:)rawhtml" + cam1Color + R"rawhtml(;border:1px solid )rawhtml" + cam1Color + R"rawhtml(55">)rawhtml" + cam1Online + R"rawhtml(</span>
      </div>
      <div class="cam-stream">)rawhtml";

  if (kamera[0].online) {
    html += "<img src='http://" + IP_CAM1 + ":81/stream' />";
  } else {
    html += "<div class='offline-msg'>⚠ Kamera Offline<br>Menunggu koneksi...</div>";
  }

  html += R"rawhtml(</div>
      <div class="cam-stats">
        <div class="stat">
          <span class="stat-label">Hama Hari Ini</span>
          <span class="stat-value">)rawhtml" + String(kamera[0].countHama) + R"rawhtml(</span>
        </div>
        <div class="stat">
          <span class="stat-label">Delta (+/-)</span>
          <span class="stat-value" style="color:)rawhtml";
  html += (kamera[0].deltaHama >= DELTA_BAHAYA) ? "#E24B4A" : "#1D9E75";
  html += R"rawhtml(">+)rawhtml" + String(kamera[0].deltaHama) + R"rawhtml(</span>
        </div>
        <div class="stat">
          <span class="stat-label">IP</span>
          <span class="stat-value" style="font-size:11px">)rawhtml" + IP_CAM1 + R"rawhtml(</span>
        </div>
      </div>
    </div>

    <!-- CAM #2 -->
    <div class="cam-card">
      <div class="cam-header">
        <span class="cam-title">📷 Kamera #2 — Yellow Trap Kanan</span>
        <span class="cam-badge" style="background:)rawhtml" + cam2Color + R"rawhtml(22;color:)rawhtml" + cam2Color + R"rawhtml(;border:1px solid )rawhtml" + cam2Color + R"rawhtml(55">)rawhtml" + cam2Online + R"rawhtml(</span>
      </div>
      <div class="cam-stream">)rawhtml";

  if (kamera[1].online) {
    html += "<img src='http://" + IP_CAM2 + ":81/stream' />";
  } else {
    html += "<div class='offline-msg'>⚠ Kamera Offline<br>Menunggu koneksi...</div>";
  }

  html += R"rawhtml(</div>
      <div class="cam-stats">
        <div class="stat">
          <span class="stat-label">Hama Hari Ini</span>
          <span class="stat-value">)rawhtml" + String(kamera[1].countHama) + R"rawhtml(</span>
        </div>
        <div class="stat">
          <span class="stat-label">Delta (+/-)</span>
          <span class="stat-value" style="color:)rawhtml";
  html += (kamera[1].deltaHama >= DELTA_BAHAYA) ? "#E24B4A" : "#1D9E75";
  html += R"rawhtml(">+)rawhtml" + String(kamera[1].deltaHama) + R"rawhtml(</span>
        </div>
        <div class="stat">
          <span class="stat-label">IP</span>
          <span class="stat-value" style="font-size:11px">)rawhtml" + IP_CAM2 + R"rawhtml(</span>
        </div>
      </div>
    </div>
  </div>

  <div class="info-grid">
    <div class="info-card">
      <div class="info-label">Total Hama Terdeteksi</div>
      <div class="info-value">)rawhtml" + String(kamera[0].countHama + kamera[1].countHama) + R"rawhtml( serangga</div>
    </div>
    <div class="info-card">
      <div class="info-label">Threshold BAHAYA</div>
      <div class="info-value">Delta &gt; )rawhtml" + String(DELTA_BAHAYA) + R"rawhtml( hama/hari</div>
    </div>
    <div class="info-card">
      <div class="info-label">Uptime Master</div>
      <div class="info-value">)rawhtml" + String(millis()/1000) + R"rawhtml( detik</div>
    </div>
  </div>

</div>

<div class="footer">
  Sistem Peringatan Dini Hama Berbasis IoT &mdash; Politeknik Manufaktur Bandung &mdash; Teknik Otomasi Mekatronika
</div>

</body>
</html>
)rawhtml";

  return html;
}

// ─────────────────────────────────────────
//  API STATUS JSON — untuk polling AJAX
// ─────────────────────────────────────────
void handleStatus() {
  String json = "{";
  json += "\"status\":\"" + String(statusSistem == BAHAYA ? "BAHAYA" : "AMAN") + "\",";
  json += "\"cam1\":{\"count\":" + String(kamera[0].countHama) +
          ",\"delta\":" + String(kamera[0].deltaHama) +
          ",\"online\":" + String(kamera[0].online ? "true" : "false") + "},";
  json += "\"cam2\":{\"count\":" + String(kamera[1].countHama) +
          ",\"delta\":" + String(kamera[1].deltaHama) +
          ",\"online\":" + String(kamera[1].online ? "true" : "false") + "}";
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ─────────────────────────────────────────
//  UPDATE AKTUATOR
// ─────────────────────────────────────────
void updateAktuator() {
  uint32_t now = millis();
  if (statusSistem == AMAN) {
    digitalWrite(PIN_BUZZER,    LOW);
    digitalWrite(PIN_LED_MERAH, LOW);
    digitalWrite(PIN_LED_HIJAU, HIGH);
  } else {
    digitalWrite(PIN_LED_MERAH, HIGH);
    digitalWrite(PIN_LED_HIJAU, LOW);
    if (now - lastBuzzerTick > (buzzerState ? 1000 : 2000)) {
      buzzerState = !buzzerState;
      digitalWrite(PIN_BUZZER, buzzerState);
      lastBuzzerTick = now;
    }
  }
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_FROM_CAM, TX_TO_CAM);

  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_LED_HIJAU, OUTPUT);
  digitalWrite(PIN_BUZZER,    LOW);
  digitalWrite(PIN_LED_MERAH, LOW);
  digitalWrite(PIN_LED_HIJAU, HIGH);

  Serial.println("\n[MASTER] Boot — Sistem Deteksi Hama");

  // Mode Access Point (Hotspot)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);

  IPAddress apIP = WiFi.softAPIP();
  Serial.println("[MASTER] Hotspot aktif!");
  Serial.println("[MASTER] SSID    : " + String(AP_SSID));
  Serial.println("[MASTER] Password: " + String(AP_PASS));
  Serial.println("[MASTER] IP      : " + apIP.toString());
  Serial.println("[MASTER] Dashboard: http://" + apIP.toString());
  Serial.println("[MASTER] Sambungkan HP ke WiFi \"" + String(AP_SSID) + "\"");

  // Route web server
  server.on("/",       [](){ server.send(200, "text/html", buildDashboard()); });
  server.on("/status", handleStatus);
  server.begin();

  Serial.println("[MASTER] Web server aktif di port 80");
  Serial.println("[MASTER] Menunggu data kamera...\n");
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  server.handleClient();

  // Baca data dari kamera via GPIO4
  while (Serial1.available()) {
    char c = Serial1.read();
    serialBuffer += c;
    if (serialBuffer.endsWith("##END##\n") ||
        serialBuffer.endsWith("##END##")) {
      parseLog(serialBuffer);
      serialBuffer = "";
      statusSebelumnya = statusSistem;
      statusSistem     = tentukanStatus();
      if (statusSistem != statusSebelumnya) {
        Serial.println("[MASTER] STATUS: " +
          String(statusSistem == BAHAYA ? "🔴 BAHAYA" : "🟢 AMAN"));
      }
    }
    if (serialBuffer.length() > 1024) serialBuffer = "";
  }

  // Aktuator
  updateAktuator();

  // Cek kamera timeout
  uint32_t now = millis();
  if (now - lastDisplay >= 30000) {
    lastDisplay = now;
    for (int i = 0; i < 2; i++) {
      if (kamera[i].online && (now - kamera[i].lastUpdate > TIMEOUT_KAMERA)) {
        kamera[i].online = false;
        Serial.printf("[MASTER] CAM#%d offline\n", i+1);
      }
    }
    Serial.printf("[MASTER] Status: %s | CAM1:%s(%d) | CAM2:%s(%d)\n",
      statusSistem == BAHAYA ? "BAHAYA" : "AMAN",
      kamera[0].online ? "ON" : "OFF", kamera[0].countHama,
      kamera[1].online ? "ON" : "OFF", kamera[1].countHama
    );
  }

  delay(10);
}
