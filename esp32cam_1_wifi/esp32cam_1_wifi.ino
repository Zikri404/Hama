/*
 * ============================================================
 *  ESP32-CAM #1 — WiFi Stream + Deteksi Hama Yellow Trap
 *  Politeknik Manufaktur Bandung | Teknik Otomasi Mekatronika
 * ============================================================
 *
 *  FITUR:
 *  - Live MJPEG stream via WiFi → port 81 /stream
 *  - Deteksi blob hama setiap CAPTURE_INTERVAL ms
 *  - Kirim log JSON ke ESP32 Master via Serial TX
 *
 *  AKSES STREAM:
 *  http://[IP-CAM1]:81/stream
 *
 *  CATATAN:
 *  Stream dan deteksi berjalan bergantian —
 *  saat stream aktif, frame dipakai untuk display.
 *  Saat interval deteksi tiba, frame diproses untuk blob.
 *
 *  Kamera connect ke hotspot ESP32 Master
 *  SSID: ESP32-CAM-System | PASS: 12345678
 * ============================================================
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPClient.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─────────────────────────────────────────
//  HOTSPOT MASTER — JANGAN DIUBAH
//  Harus sama dengan setting di esp32_master
// ─────────────────────────────────────────
#define WIFI_SSID      "ESP32-CAM-System"
#define WIFI_PASS      "12345678"
#define MASTER_IP      "192.168.4.1"   // IP ESP32 Master
#define MASTER_PORT    80

// ─────────────────────────────────────────
//  KONFIGURASI
// ─────────────────────────────────────────
#define CAM_ID            1
#define MASTER_IP   "192.168.4.1"  // IP ESP32 Master
#define CAPTURE_INTERVAL  30000    // ms — interval deteksi hama
#define MIN_BLOB_SIZE     8
#define MAX_BLOB_SIZE     500
#define DELTA_ALERT       10

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
httpd_handle_t stream_httpd = NULL;
int      countHariIni  = 0;
int      countKemarin  = 0;
bool     cameraReady   = false;
bool     wifiReady     = false;
uint32_t lastCapture   = 0;
uint32_t lastHeartbeat = 0;
String   cmdBuffer     = "";

uint8_t* maskBuffer = nullptr;
bool*    visited    = nullptr;

// ─────────────────────────────────────────
//  INIT KAMERA — MODE JPEG UNTUK STREAM
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
  config.pixel_format = PIXFORMAT_JPEG;  // JPEG untuk stream

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

  Serial.printf("[CAM%d] Kamera siap ✓ (%s)\n", CAM_ID,
    psramFound() ? "VGA" : "QVGA");
  return true;
}

// ─────────────────────────────────────────
//  MJPEG STREAM HANDLER
// ─────────────────────────────────────────
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t streamHandler(httpd_req_t* req) {
  char part_buf[64];
  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;

    httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    httpd_resp_send_chunk(req, part_buf, hlen);
    esp_err_t res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return ESP_OK;
}

void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port   = 32769;

  httpd_uri_t stream_uri = {
    "/stream", HTTP_GET, streamHandler, NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.printf("[CAM%d] Stream: http://%s:81/stream\n",
      CAM_ID, WiFi.localIP().toString().c_str());
  }
}

// ─────────────────────────────────────────
//  DETEKSI HAMA — AMBIL FRAME RGB565 TERPISAH
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

int jalankanDeteksi() {
  // Flush frame buffer lama sebelum switch format (fix FB-OVF)
  camera_fb_t* flush;
  for (int i = 0; i < 3; i++) {
    flush = esp_camera_fb_get();
    if (flush) esp_camera_fb_return(flush);
    delay(30);
  }

  // Ganti sementara ke RGB565 untuk deteksi
  sensor_t* s = esp_camera_sensor_get();
  s->set_pixformat(s, PIXFORMAT_RGB565);
  s->set_framesize(s, FRAMESIZE_QVGA);
  delay(200);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    // Kembalikan ke JPEG
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, psramFound() ? FRAMESIZE_VGA : FRAMESIZE_QVGA);
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
  esp_camera_fb_return(fb);

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

  // Kembalikan ke JPEG untuk stream
  s->set_pixformat(s, PIXFORMAT_JPEG);
  s->set_framesize(s, psramFound() ? FRAMESIZE_VGA : FRAMESIZE_QVGA);
  delay(100);

  return blobs;
}

// ─────────────────────────────────────────
//  KIRIM LOG KE MASTER VIA HTTP POST
// ─────────────────────────────────────────
void kirimLog(int count, int delta) {
  if (!wifiReady) return;

  String status = "AMAN";
  if (delta >= DELTA_ALERT) status = "BAHAYA";

  String json = "{\"cam\":" + String(CAM_ID) +
                ",\"count\":" + String(count) +
                ",\"delta\":" + String(delta) +
                ",\"status\":\"" + status + "\"" +
                ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
                ",\"ts\":" + String(millis()/1000) + "}";

  HTTPClient http;
  String url = "http://" + String(MASTER_IP) + ":" + String(MASTER_PORT) + "/log";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);

  if (code > 0) {
    Serial.printf("[CAM%d] Log terkirim ke master → HTTP %d\n", CAM_ID, code);
  } else {
    Serial.printf("[CAM%d] Gagal kirim log: %s\n", CAM_ID, http.errorToString(code).c_str());
  }
  http.end();
}

// ─────────────────────────────────────────
//  KIRIM HEARTBEAT KE MASTER VIA HTTP
// ─────────────────────────────────────────
void kirimHeartbeat() {
  if (!wifiReady) return;
  HTTPClient http;
  String url = "http://" + String(MASTER_IP) + ":" + String(MASTER_PORT) +
               "/hb?cam=" + String(CAM_ID);
  http.begin(url);
  http.GET();
  http.end();
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(500);

  Serial.printf("\n[CAM%d] Boot...\n", CAM_ID);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Alokasi buffer deteksi
  maskBuffer = (uint8_t*)malloc(320 * 240);
  visited    = (bool*)malloc(320 * 240 * sizeof(bool));
  if (!maskBuffer || !visited) {
    Serial.println("[ERROR] Alokasi buffer gagal!");
    while(1) delay(1000);
  }

  // Init kamera
  cameraReady = initCamera();
  if (!cameraReady) { delay(3000); ESP.restart(); }

  // Konek WiFi
  Serial.printf("[CAM%d] Konek ke WiFi: %s\n", CAM_ID, WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(500); Serial.print("."); attempt++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiReady = true;
    Serial.printf("[CAM%d] WiFi OK — IP: %s\n",
      CAM_ID, WiFi.localIP().toString().c_str());
    startStreamServer();
  } else {
    Serial.printf("[CAM%d] WiFi GAGAL — stream tidak tersedia\n", CAM_ID);
    Serial.printf("[CAM%d] Deteksi tetap jalan via Serial\n", CAM_ID);
  }

  // Kirim status boot ke master
  delay(500);
  Serial.printf("##LOG##{"
    "\"cam\":%d,\"count\":0,\"delta\":0,"
    "\"status\":\"READY\",\"ip\":\"%s\",\"ts\":0"
    "}##END##\n",
    CAM_ID, WiFi.localIP().toString().c_str()
  );

  lastCapture = millis();
  Serial.printf("[CAM%d] Siap. Deteksi pertama dalam 30 detik...\n\n", CAM_ID);
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // Deteksi hama setiap interval
  if (now - lastCapture >= CAPTURE_INTERVAL) {
    lastCapture = now;
    Serial.printf("[CAM%d] Mulai deteksi...\n", CAM_ID);

    int jumlah = jalankanDeteksi();
    if (jumlah >= 0) {
      if (jumlah > countHariIni) countHariIni = jumlah;
      int delta = max(0, countHariIni - countKemarin);

      Serial.printf("[CAM%d] Hasil: %d hama | Delta: +%d\n",
                    CAM_ID, countHariIni, delta);
      kirimLog(countHariIni, delta);
    }
  }

  // Heartbeat ke master via HTTP
  if (now - lastHeartbeat >= 10000) {
    lastHeartbeat = now;
    kirimHeartbeat();
  }

  // Reconnect WiFi jika putus
  if (wifiReady && WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  delay(50);
}
