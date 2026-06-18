/*
 * ============================================================
 * ESP32 MASTER — Luring (Offline) + 2 Kamera via UART
 * Politeknik Manufaktur Bandung | Teknik Otomasi Mekatronika
 * ============================================================
 *
 * FUNGSI:
 * - Terima log dari CAM#1 via HardwareSerial2 (GPIO27 RX)
 * - Kontrol buzzer & LED sesuai status AMAN/BAHAYA
 *
 * WIRING:
 * CAM#1 TX (pin15) → ESP32 Master GPIO27 (RX)   ← UART2
 * GND CAM#1        → GND Master (wajib)
 *
 * CATATAN:
 * CAM#2 tidak langsung ke Master. Data CAM#2 diteruskan
 * oleh CAM#1 sebagai relay dalam satu jalur UART yang sama.
 * Master membaca semua data (CAM#1 & CAM#2) dari GPIO27.
 * ============================================================
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────
//  KONFIGURASI SERIAL
// ─────────────────────────────────────────
HardwareSerial SerialCam1(2);   // UART2 — dari CAM#1 (+ relay CAM#2)

#define RX_CAM1  27   // CAM#1 pin15 → GPIO27
#define TX_CAM1  14   // tidak dipakai, wajib define

// ─────────────────────────────────────────
//  KONFIGURASI SISTEM
// ─────────────────────────────────────────
#define DELTA_BAHAYA    10
#define TIMEOUT_KAMERA  120000

#define PIN_BUZZER     25
#define PIN_LED_MERAH  26
#define PIN_LED_HIJAU   4

// ─────────────────────────────────────────
//  DATA KAMERA
// ─────────────────────────────────────────
struct KameraData {
  int      id;
  int      countHama;
  int      deltaHama;
  String   status;
  uint32_t lastUpdate;
  bool     online;
};

KameraData kamera[2] = {
  {1, 0, 0, "BOOT", 0, false},
  {2, 0, 0, "BOOT", 0, false}
};

// ─────────────────────────────────────────
//  STATUS SISTEM
// ─────────────────────────────────────────
enum StatusSistem { AMAN, BAHAYA };
StatusSistem statusSistem     = AMAN;
StatusSistem statusSebelumnya = AMAN;

// Buffer baca dari CAM#1
String bufferCam1   = "";
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
  if (deserializeJson(doc, json) != DeserializationError::Ok) {
    Serial.println("[MASTER] Gagal parsing JSON");
    return;
  }

  int    camId = doc["cam"]    | 0;
  int    count = doc["count"]  | 0;
  int    delta = doc["delta"]  | 0;
  String stat  = doc["status"] | "UNKNOWN";

  if (camId < 1 || camId > 2) return;
  int idx = camId - 1;

  kamera[idx].countHama  = count;
  kamera[idx].deltaHama  = delta;
  kamera[idx].status     = stat;
  kamera[idx].lastUpdate = millis();
  kamera[idx].online     = true;

  Serial.printf("[MASTER] <- CAM#%d: %d hama | Delta: +%d | %s\n",
    camId, count, delta, stat.c_str());

  // Update status sistem
  statusSebelumnya = statusSistem;
  statusSistem     = tentukanStatus();
  if (statusSistem != statusSebelumnya) {
    Serial.println(statusSistem == BAHAYA
      ? "[MASTER] STATUS: BAHAYA"
      : "[MASTER] STATUS: AMAN");
  }
}

// ─────────────────────────────────────────
//  TENTUKAN STATUS SISTEM
// ─────────────────────────────────────────
StatusSistem tentukanStatus() {
  int maxDelta = max(kamera[0].deltaHama, kamera[1].deltaHama);
  return (maxDelta >= DELTA_BAHAYA) ? BAHAYA : AMAN;
}

// ─────────────────────────────────────────
//  BACA SERIAL DARI CAM#1
// ─────────────────────────────────────────
void bacaSerial(HardwareSerial &ser, String &buf) {
  while (ser.available()) {
    char c = ser.read();
    buf += c;

    if (buf.endsWith("##END##\n") || buf.endsWith("##END##")) {
      parseLog(buf);
      buf = "";
    }

    // Anti overflow
    if (buf.length() > 1024) buf = "";
  }
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

  // Serial dari CAM#1 (UART2, GPIO27 RX)
  SerialCam1.begin(115200, SERIAL_8N1, RX_CAM1, TX_CAM1);

  pinMode(PIN_BUZZER,    OUTPUT);
  pinMode(PIN_LED_MERAH, OUTPUT);
  pinMode(PIN_LED_HIJAU, OUTPUT);

  digitalWrite(PIN_BUZZER,    LOW);
  digitalWrite(PIN_LED_MERAH, LOW);
  digitalWrite(PIN_LED_HIJAU, HIGH);

  Serial.println("\n[MASTER] Boot — Sistem Deteksi Hama Luring");
  Serial.println("[MASTER] Menunggu data dari CAM#1 via GPIO27...\n");
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  // Baca dari CAM#1 (termasuk relay data CAM#2)
  bacaSerial(SerialCam1, bufferCam1);

  // Aktuator
  updateAktuator();

  // Laporan status setiap 30 detik
  uint32_t now = millis();
  if (now - lastDisplay >= 30000) {
    lastDisplay = now;
    for (int i = 0; i < 2; i++) {
      if (kamera[i].online && (now - kamera[i].lastUpdate > TIMEOUT_KAMERA)) {
        kamera[i].online = false;
        Serial.printf("[MASTER] CAM#%d offline\n", i + 1);
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
