/*
 * ============================================================
 *  ESP32-CAM #1 — Deteksi Hama + RELAY data dari CAM#2
 *  Politeknik Manufaktur Bandung | Teknik Otomasi Mekatronika
 * ============================================================
 *
 *  FUNGSI:
 *  - Deteksi blob hama setiap 30 detik (tugas sendiri)
 *  - Terima data dari CAM#2 via Serial2 (pin14 RX)
 *  - Teruskan data CAM#2 + kirim data sendiri ke Master via SerialMaster
 *
 *  WIRING:
 *  CAM#2 TX (pin15) → CAM#1 pin14 (RX)
 *  CAM#1 TX (pin15) → ESP32 Master GPIO27 (RX)   ← UART1
 *  CAM#1 GND        → GND CAM#2 & GND Master
 *
 *  SERIAL:
 *  Serial      (UART0, pin1)  = debug monitor IDE saja
 *  SerialMaster(UART1, pin15) = kirim ##LOG## ke Master
 *  SerialDariCam2(UART2, pin14) = terima dari CAM#2
 * ============================================================
 */

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─────────────────────────────────────────
//  KONFIGURASI
// ─────────────────────────────────────────
#define CAM_ID            1
#define CAPTURE_INTERVAL  30000
#define MIN_BLOB_SIZE     8
#define MAX_BLOB_SIZE     500
#define DELTA_ALERT       10

// Serial ke Master (UART1, TX = pin15)
HardwareSerial SerialMaster(1);
#define TX_KE_MASTER  15
#define RX_DUMMY_M    -1

// Serial dari CAM#2 (UART2, RX = pin14)
HardwareSerial SerialDariCam2(2);
#define RX_DARI_CAM2  14
#define TX_DUMMY      -1

// ─────────────────────────────────────────
//  PIN KAMERA AI THINKER
// ─────────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22
#define FLASH_LED_PIN    4

// ─────────────────────────────────────────
//  RANGE WARNA KUNING TRAP
// ─────────────────────────────────────────
#define YELLOW_R_MIN  140
#define YELLOW_G_MIN  120
#define YELLOW_B_MAX   80

// ─────────────────────────────────────────
//  VARIABEL GLOBAL
// ─────────────────────────────────────────
int      countHariIni  = 0;
int      countKemarin  = 0;
bool     cameraReady   = false;
uint32_t lastCapture   = 0;

uint8_t* maskBuffer = nullptr;
bool*    visited    = nullptr;

// Buffer relay dari CAM#2
String bufferCam2 = "";

// ─────────────────────────────────────────
//  INIT KAMERA
// ─────────────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_D0; config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2; config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4; config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6; config.pin_d7 = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.printf("[CAM%d] Init gagal!\n", CAM_ID);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, 1);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_gain_ctrl(s, 1);

  Serial.printf("[CAM%d] Kamera siap ✓\n", CAM_ID);
  return true;
}

// ─────────────────────────────────────────
//  DETEKSI WARNA
// ─────────────────────────────────────────
void rgb565ToRGB(uint16_t p, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((p >> 11) & 0x1F) * 255 / 31;
  g = ((p >> 5)  & 0x3F) * 255 / 63;
  b = ( p        & 0x1F) * 255 / 31;
}

bool isYellow(uint8_t r, uint8_t g, uint8_t b) {
  return (r > YELLOW_R_MIN && g > YELLOW_G_MIN && b < YELLOW_B_MAX);
}

int floodFill(int sx, int sy, int w, int h) {
  static int qx[1000], qy[1000];
  int top = 0, count = 0;
  qx[top] = sx; qy[top] = sy; top++;
  while (top > 0) {
    top--;
    int x = qx[top], y = qy[top];
    if (x < 0 || x >= w || y < 0 || y >= h) continue;
    int idx = y * w + x;
    if (visited[idx] || !maskBuffer[idx]) continue;
    visited[idx] = true; count++;
    if (top < 996) {
      if (x+1 < w) { qx[top]=x+1; qy[top]=y;   top++; }
      if (x-1 >= 0){ qx[top]=x-1; qy[top]=y;   top++; }
      if (y+1 < h) { qx[top]=x;   qy[top]=y+1; top++; }
      if (y-1 >= 0){ qx[top]=x;   qy[top]=y-1; top++; }
    }
  }
  return count;
}

// ─────────────────────────────────────────
//  DETEKSI HAMA
// ─────────────────────────────────────────
int jalankanDeteksi() {
  Serial.printf("[CAM%d] Flash ON\n", CAM_ID);
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(150);

  camera_fb_t* fb = esp_camera_fb_get();

  digitalWrite(FLASH_LED_PIN, LOW);
  Serial.printf("[CAM%d] Flash OFF\n", CAM_ID);

  if (!fb) {
    Serial.println("[CAM] Gagal ambil frame!");
    return -1;
  }

  int w = fb->width, h = fb->height;
  uint16_t* px = (uint16_t*)fb->buf;
  memset(maskBuffer, 0, w * h);
  memset(visited,    0, w * h * sizeof(bool));

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint16_t p = px[y*w+x];
      p = (p >> 8) | (p << 8);
      uint8_t r, g, b;
      rgb565ToRGB(p, r, g, b);
      if (!isYellow(r, g, b)) maskBuffer[y*w+x] = 1;
    }
  }

  int blobs = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx = y * w + x;
      if (maskBuffer[idx] && !visited[idx]) {
        int sz = floodFill(x, y, w, h);
        if (sz >= MIN_BLOB_SIZE && sz <= MAX_BLOB_SIZE) blobs++;
      }
    }
  }

  esp_camera_fb_return(fb);
  return blobs;
}

// ─────────────────────────────────────────
//  KIRIM LOG CAM#1 KE MASTER
//  Menggunakan SerialMaster (UART1, pin15) — BUKAN Serial debug
// ─────────────────────────────────────────
void kirimLogKeMaster(int count, int delta) {
  String status = (delta >= DELTA_ALERT) ? "BAHAYA" : "AMAN";

  SerialMaster.printf("##LOG##{"
    "\"cam\":%d,"
    "\"count\":%d,"
    "\"delta\":%d,"
    "\"status\":\"%s\","
    "\"ip\":\"KABEL_UART\","
    "\"ts\":%lu"
    "}##END##\n",
    CAM_ID, count, delta, status.c_str(), millis() / 1000
  );

  // Debug ke IDE monitor (Serial, pin1) — tidak ganggu Master
  Serial.printf("[CAM%d] Log dikirim ke Master: %d hama | Δ+%d | %s\n",
    CAM_ID, count, delta, status.c_str());
}

// ─────────────────────────────────────────
//  RELAY DATA DARI CAM#2 KE MASTER
//  Terima dari SerialDariCam2 (pin14), teruskan via SerialMaster (pin15)
// ─────────────────────────────────────────
void relayDataCam2() {
  while (SerialDariCam2.available()) {
    char c = SerialDariCam2.read();
    bufferCam2 += c;

    if (bufferCam2.endsWith("##END##\n") ||
        bufferCam2.endsWith("##END##")) {

      // Teruskan ke Master via SerialMaster (bukan Serial debug)
      SerialMaster.print(bufferCam2);
      Serial.printf("[CAM1-RELAY] Data CAM#2 diteruskan ke Master\n");
      bufferCam2 = "";
    }

    // Anti overflow
    if (bufferCam2.length() > 1024) bufferCam2 = "";
  }
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Serial debug ke IDE (UART0, pin1) — jangan dikabel ke Master
  Serial.begin(115200);
  delay(500);

  // SerialMaster ke Master (UART1, TX = pin15) → Master GPIO27
  SerialMaster.begin(115200, SERIAL_8N1, RX_DUMMY_M, TX_KE_MASTER);

  // Serial dari CAM#2 (UART2, RX = pin14)
  SerialDariCam2.begin(115200, SERIAL_8N1, RX_DARI_CAM2, TX_DUMMY);

  Serial.printf("\n[CAM%d] Boot — Mode Relay\n", CAM_ID);
  Serial.println("[CAM1] SerialMaster → pin15 (TX ke Master GPIO27)");
  Serial.println("[CAM1] SerialDariCam2 ← pin14 (RX dari CAM#2)");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Alokasi buffer
  maskBuffer = (uint8_t*)malloc(320 * 240);
  visited    = (bool*)malloc(320 * 240 * sizeof(bool));
  if (!maskBuffer || !visited) {
    Serial.println("[ERROR] Alokasi buffer gagal!");
    while(1) delay(1000);
  }

  // Init kamera
  cameraReady = initCamera();
  if (!cameraReady) { delay(3000); ESP.restart(); }

  // Kirim boot log ke Master via SerialMaster
  delay(500);
  SerialMaster.printf("##LOG##{"
    "\"cam\":1,\"count\":0,\"delta\":0,"
    "\"status\":\"READY\",\"ip\":\"KABEL\",\"ts\":0"
    "}##END##\n"
  );
  Serial.println("[CAM1] Boot log dikirim ke Master");

  lastCapture = millis();
  Serial.printf("[CAM%d] Siap. Deteksi + relay aktif...\n\n", CAM_ID);
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // Relay data dari CAM#2 ke Master (prioritas tinggi)
  relayDataCam2();

  // Deteksi hama sendiri setiap interval
  if (now - lastCapture >= CAPTURE_INTERVAL) {
    lastCapture = now;
    Serial.printf("[CAM%d] Mulai deteksi...\n", CAM_ID);

    int jumlah = jalankanDeteksi();
    if (jumlah >= 0) {
      countHariIni = jumlah;
      int delta = max(0, countHariIni - countKemarin);
      Serial.printf("[CAM%d] Hasil: %d hama | Delta: +%d\n",
                    CAM_ID, countHariIni, delta);
      kirimLogKeMaster(countHariIni, delta);
    }
  }

  delay(10);
}
