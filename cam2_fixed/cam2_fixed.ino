/*
 * ============================================================
 *  ESP32-CAM #2 — Deteksi Hama + Kirim ke CAM#1 (Relay)
 *  Politeknik Manufaktur Bandung | Teknik Otomasi Mekatronika
 * ============================================================
 *
 *  FUNGSI:
 *  - Deteksi blob hama setiap 30 detik
 *  - Kirim hasil ke CAM#1 via UART1 pin15 (UOT)
 *  - CAM#1 yang meneruskan ke Master
 *
 *  WIRING:
 *  CAM#2 TX pin15 (UOT) → CAM#1 pin14 (RX)
 *  CAM#2 GND             → GND CAM#1
 *  CAM#2 5V              → 5V (power terpisah atau dari master)
 *
 *  SERIAL:
 *  Serial      (UART0, pin1)  = debug monitor IDE saja (USB)
 *  SerialKeCAM1(UART1, pin15) = kirim ##LOG## ke CAM#1
 *
 *  FIX: Sebelumnya pakai Serial.printf() yang keluar di pin1 (USB),
 *       sekarang pakai SerialKeCAM1.printf() yang keluar di pin15.
 * ============================================================
 */

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─────────────────────────────────────────
//  KONFIGURASI
// ─────────────────────────────────────────
#define CAM_ID            2
#define CAPTURE_INTERVAL  30000
#define MIN_BLOB_SIZE     8
#define MAX_BLOB_SIZE     500
#define DELTA_ALERT       10

// ─────────────────────────────────────────
//  SERIAL KE CAM#1 (UART1, TX = pin15)
//  pin15 (UOT) → CAM#1 pin14 (RX)
// ─────────────────────────────────────────
HardwareSerial SerialKeCAM1(1);   // UART1
#define TX_KE_CAM1   15           // pin15 = UOT ESP32-CAM
#define RX_DUMMY     -1           // tidak butuh RX

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
    Serial.println("[CAM2] Gagal ambil frame!");
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
//  KIRIM LOG KE CAM#1 (RELAY)
//  SerialKeCAM1 TX (pin15/UOT) → CAM#1 pin14 (RX)
//  FIX: dulu pakai Serial.printf() → keluar di pin1 (USB, tidak sampai CAM#1)
//       sekarang pakai SerialKeCAM1.printf() → keluar di pin15 (sampai CAM#1)
// ─────────────────────────────────────────
void kirimKeCAM1(int count, int delta) {
  String status = (delta >= DELTA_ALERT) ? "BAHAYA" : "AMAN";

  // Kirim ke CAM#1 via pin15 (UART1)
  SerialKeCAM1.printf("##LOG##{"
    "\"cam\":%d,"
    "\"count\":%d,"
    "\"delta\":%d,"
    "\"status\":\"%s\","
    "\"ip\":\"KABEL_RELAY\","
    "\"ts\":%lu"
    "}##END##\n",
    CAM_ID, count, delta, status.c_str(), millis() / 1000
  );

  // Debug ke Serial Monitor IDE (pin1/USB) — tidak ganggu jalur ke CAM#1
  Serial.printf("[CAM%d] Kirim ke CAM1: %d hama | Δ+%d | %s\n",
    CAM_ID, count, delta, status.c_str());
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Serial debug ke IDE (UART0, pin1) — JANGAN dikabel ke CAM#1
  Serial.begin(115200);
  delay(500);

  // SerialKeCAM1 ke CAM#1 (UART1, TX = pin15)
  SerialKeCAM1.begin(115200, SERIAL_8N1, RX_DUMMY, TX_KE_CAM1);

  Serial.printf("\n[CAM%d] Boot — Mode Relay ke CAM#1\n", CAM_ID);
  Serial.println("[CAM2] SerialKeCAM1 → pin15 (TX ke CAM#1 pin14)");

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

  // Kirim boot log ke CAM#1 via pin15
  delay(500);
  SerialKeCAM1.printf("##LOG##{"
    "\"cam\":2,\"count\":0,\"delta\":0,"
    "\"status\":\"READY\",\"ip\":\"KABEL_RELAY\",\"ts\":0"
    "}##END##\n"
  );
  Serial.println("[CAM2] Boot log dikirim ke CAM#1 via pin15");

  lastCapture = millis();
  Serial.printf("[CAM%d] Siap. Kirim ke CAM#1 setiap %d detik...\n\n",
    CAM_ID, CAPTURE_INTERVAL / 1000);
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  if (now - lastCapture >= CAPTURE_INTERVAL) {
    lastCapture = now;
    Serial.printf("[CAM%d] Mulai deteksi...\n", CAM_ID);

    int jumlah = jalankanDeteksi();
    if (jumlah >= 0) {
      countKemarin  = countHariIni;   // simpan sebelum update
      countHariIni  = jumlah;
      int delta = max(0, countHariIni - countKemarin);
      Serial.printf("[CAM%d] Hasil: %d hama | Delta: +%d\n",
                    CAM_ID, countHariIni, delta);
      kirimKeCAM1(countHariIni, delta);
    }
  }

  delay(10);
}
