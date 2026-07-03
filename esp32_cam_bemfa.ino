/*
  ESP32-CAM：巴法云 HTTP 图片上传 + 微信通知版本
  作用：替换原 OneNET MQTT 部分。

  通信协议保持不变：
  ESP32 主控 -> ESP32-CAM：
    EVENT:DOORBELL
    EVENT:FALSE_TRIGGER,VALID=5/30
    EVENT:ARRIVED,FSR=2450,DIST=13.6,VALID=23/30
    EVENT:REMOVED,FSR=320,DIST=72.5,VALID=17/20
    CMD:SHUTDOWN
    CMD:PING

  ESP32-CAM -> ESP32 主控：
    CAM_READY
    ACK:DOORBELL
    ACK:FALSE_TRIGGER
    ACK:ARRIVED
    ACK:REMOVED
    CAM_SLEEP_OK
    ACK:PING
*/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// 不同 Arduino-ESP32 版本一般会在 SD_MMC.h 里定义这些频率宏。
// 这里加兜底，避免旧环境缺宏导致编译失败。单位为 kHz。
#ifndef SDMMC_FREQ_DEFAULT
#define SDMMC_FREQ_DEFAULT 20000
#endif
#ifndef SDMMC_FREQ_PROBING
#define SDMMC_FREQ_PROBING 400
#endif

// ===================== 1. 需要修改的配置 =====================
const char* WIFI_SSID = "TempH";
const char* WIFI_PASS = "1123123456";

// 巴法云控制台首页获取的用户私钥 UID
const char* BEMFA_UID = "4f381d55d5574c0ba7d0486e32d4fad3";

// 你已经创建的图片主题
const char* BEMFA_IMAGE_TOPIC = "packagepic";

// 微信通知里显示的设备名称
const char* BEMFA_DEVICE_NAME = "包裹投递监控";

// 设置为 1：上电后自动发送一条微信测试消息，并拍照上传一次。
// 单独测试 ESP32-CAM 时可改为 1；双板联调时建议保持 0。
#define RUN_STANDALONE_TEST_ON_BOOT 0

// ===================== 本地 SD 卡网页配置 =====================
// 本地浏览器访问 ESP32-CAM 的 IP 即可查看记录。
// 页面尽量简单，只保留首页、历史记录、图片列表和图片查看。
const char* PHOTO_DIR = "/photos";
const char* RECORDS_FILE = "/records.csv";
const int MAX_RECORDS = 10;

// SD_MMC 参数说明：
// Arduino-ESP32 当前常用接口顺序为：
// begin(挂载点, 1bit模式, 挂载失败是否格式化, 频率kHz, 最大打开文件数)
// 你原代码把“5”和“10000000”写反了，等于让 SD 以极低/异常频率初始化，
// 这正是联调日志里 send_op_cond 0x107、sdOK=false 的主要代码漏洞。
#define SD_MMC_MAX_OPEN_FILES   5
#define SD_MMC_MAIN_FREQ_KHZ    SDMMC_FREQ_DEFAULT   // 常用默认频率，通常 20MHz
#define SD_MMC_SAFE_FREQ_KHZ    SDMMC_FREQ_PROBING   // 兜底低速，通常 400kHz
#define SD_INIT_RETRY_COUNT     3
#define SD_REINIT_DELAY_MS      600

// ===================== 2. 巴法云接口 =====================
const char* BEMFA_IMAGE_UPLOAD_URL = "http://apis.bemfa.com/vb/api/v1/imagesUploadBin";
const char* BEMFA_WECHAT_ALERT_URL = "http://apis.bemfa.com/vb/wechat/v1/wechatAlertJson";

// ===================== 3. AI Thinker ESP32-CAM 摄像头引脚 =====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ===================== 闪光灯配置 =====================
// AI Thinker ESP32-CAM 板载白光 LED 一般接 GPIO4。
// 当前 SD 卡使用 1-bit 模式，因此不会占用 GPIO4。
#define FLASH_LED_PIN       4
#define USE_FLASH_LED       1
#define FLASH_PRELIGHT_MS   180

// ===================== 4. 全局状态 =====================
bool cameraOK = false;
bool sdOK = false;

WebServer server(80);

struct PhotoResult {
  bool captured;
  bool savedToSD;
  bool uploaded;
  String sdPath;
  String imageUrl;
  int imageLength;
  int httpCode;
  String serverResponse;
};

// ===================== 5. 字符串工具 =====================
String jsonEscape(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\n", "\\n");
  s.replace("\r", "");
  return s;
}

String getMetaFromCommand(const String& cmd) {
  int commaIndex = cmd.indexOf(',');
  if (commaIndex < 0) {
    return "";
  }
  return cmd.substring(commaIndex + 1);
}

String extractImageUrl(const String& response) {
  int keyIndex = response.indexOf("\"url\"");
  if (keyIndex < 0) return "";

  int colonIndex = response.indexOf(':', keyIndex);
  if (colonIndex < 0) return "";

  int firstQuote = response.indexOf('"', colonIndex + 1);
  if (firstQuote < 0) return "";

  int secondQuote = response.indexOf('"', firstQuote + 1);
  if (secondQuote < 0) return "";

  return response.substring(firstQuote + 1, secondQuote);
}

String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

String csvSafe(String s) {
  s.replace("\r", " ");
  s.replace("\n", " ");
  s.replace(",", ";");
  return s;
}

String uploadStatusText(bool uploaded) {
  return uploaded ? "OK" : "UPLOAD_FAILED";
}

String makeSimpleTime() {
  // 不做固定 IP 和 NTP 时钟处理，先用运行毫秒数作为记录时间。
  return String(millis());
}

bool safePhotoPath(const String& path) {
  return path.startsWith(String(PHOTO_DIR) + "/") && path.indexOf("..") < 0 && path.endsWith(".jpg");
}

void flashOn() {
#if USE_FLASH_LED
  digitalWrite(FLASH_LED_PIN, HIGH);
  Serial.println("[FLASH] on");
#endif
}

void flashOff() {
#if USE_FLASH_LED
  digitalWrite(FLASH_LED_PIN, LOW);
  Serial.println("[FLASH] off");
#endif
}

// ===================== 6. WiFi =====================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.println("[WiFi] connecting...");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] connected, IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] connect failed");
  }
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  connectWiFi();
  return WiFi.status() == WL_CONNECTED;
}

// ===================== 7. 巴法云微信通知 =====================
bool sendBemfaWechat(const String& message, const String& url = "") {
  if (!ensureWiFi()) {
    Serial.println("[BEMFA] WeChat failed: WiFi not connected");
    return false;
  }

  HTTPClient http;
  http.setTimeout(15000);

  if (!http.begin(BEMFA_WECHAT_ALERT_URL)) {
    Serial.println("[BEMFA] http begin failed: wechat");
    return false;
  }

  http.addHeader("Content-Type", "application/json; charset=utf-8");

  String body = "{";
  body += String("\"uid\":\"") + jsonEscape(String(BEMFA_UID)) + "\",";
  body += String("\"device\":\"") + jsonEscape(String(BEMFA_DEVICE_NAME)) + "\",";
  body += String("\"message\":\"") + jsonEscape(message) + "\",";
  body += "\"group\":\"package\"";
  if (url.length() > 0) {
    body += String(",\"url\":\"") + jsonEscape(url) + "\"";
  }
  body += "}";

  Serial.println("[BEMFA] WeChat request:");
  Serial.println(body);

  int code = http.POST(body);
  String resp = http.getString();

  Serial.print("[BEMFA] WeChat HTTP code=");
  Serial.println(code);
  Serial.print("[BEMFA] WeChat response=");
  Serial.println(resp);

  http.end();

  return code > 0 && code < 400;
}

// ===================== 8. 摄像头初始化 =====================
bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;

  // ESP32 Arduino Core 2.x / 3.x 使用 pin_sccb_sda / pin_sccb_scl。
  // 如果你的环境提示没有该成员，把 sccb 改成 sscb 即可。
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.print("[CAMERA] init failed, error=0x");
    Serial.println(err, HEX);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
  }

  Serial.println("[CAMERA] init ok");
  return true;
}

// ===================== 9. SD 卡初始化，1-bit 模式 =====================
void prepareSDPins() {
  // AI Thinker ESP32-CAM 的 SD_MMC 固定引脚：
  // CLK=GPIO14, CMD=GPIO15, D0=GPIO2。
  // 1-bit 模式只使用这三根数据线，但官方说明 D3 仍需要上拉，
  // 否则部分卡会误入 SPI 模式，所以这里同时给 GPIO13 开内部上拉。
  // 不要在这里动 GPIO4：本项目 GPIO4 用作闪光灯，且 1-bit 模式不使用 D1。
  pinMode(14, INPUT_PULLUP);  // SD CLK
  pinMode(15, INPUT_PULLUP);  // SD CMD
  pinMode(2,  INPUT_PULLUP);  // SD D0，注意不要外接下拉
  pinMode(13, INPUT_PULLUP);  // SD D3/CS，上拉防止卡进入 SPI 模式
  delay(80);
}

bool beginSDMMCAtFreq(uint32_t freqKHz) {
  // 关键修正点：第 4 个参数是频率，第 5 个参数才是最大打开文件数。
  // 原来写成 SD_MMC.begin("/sdcard", true, false, 5, 10000000) 是错误的。
  return SD_MMC.begin("/sdcard", true, false, freqKHz, SD_MMC_MAX_OPEN_FILES);
}

bool sdWriteSelfTest() {
  const char* testPath = "/sd_selftest.txt";
  const char* marker = "ESP32_CAM_SD_OK";

  File wf = SD_MMC.open(testPath, FILE_WRITE);
  if (!wf) {
    Serial.println("[SD] self-test open write failed");
    return false;
  }
  size_t written = wf.print(marker);
  wf.flush();
  wf.close();

  if (written != strlen(marker)) {
    Serial.println("[SD] self-test write length mismatch");
    return false;
  }

  File rf = SD_MMC.open(testPath, FILE_READ);
  if (!rf) {
    Serial.println("[SD] self-test open read failed");
    return false;
  }
  String content = rf.readString();
  rf.close();

  SD_MMC.remove(testPath);

  if (content.indexOf(marker) < 0) {
    Serial.println("[SD] self-test read content mismatch");
    return false;
  }

  Serial.println("[SD] self-test write/read ok");
  return true;
}

bool initSDCardOnce(uint32_t freqKHz, const char* label) {
  prepareSDPins();

  Serial.print("[SD] begin 1-bit, freqKHz=");
  Serial.print(freqKHz);
  Serial.print(", profile=");
  Serial.println(label);

  if (!beginSDMMCAtFreq(freqKHz)) {
    Serial.println("[SD] SD_MMC begin failed");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD] no SD card");
    SD_MMC.end();
    return false;
  }

  Serial.print("[SD] card type=");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  Serial.print("[SD] cardSizeMB=");
  Serial.println((uint32_t)(SD_MMC.cardSize() / (1024 * 1024)));

  if (!SD_MMC.exists(PHOTO_DIR)) {
    if (SD_MMC.mkdir(PHOTO_DIR)) {
      Serial.println("[SD] /photos created");
    } else {
      Serial.println("[SD] /photos create failed");
      SD_MMC.end();
      return false;
    }
  }

  if (!SD_MMC.exists(RECORDS_FILE)) {
    File f = SD_MMC.open(RECORDS_FILE, FILE_WRITE);
    if (f) {
      f.println("event,time,meta,local_path,cloud_url,upload_status");
      f.flush();
      f.close();
    } else {
      Serial.println("[SD] records.csv create failed");
      SD_MMC.end();
      return false;
    }
  }

  if (!sdWriteSelfTest()) {
    Serial.println("[SD] mount ok but write/read self-test failed");
    SD_MMC.end();
    return false;
  }

  Serial.println("[SD] SD card init ok, 1-bit mode");
  return true;
}

bool initSDCard() {
  // 先用默认频率；如果联调电源/走线导致失败，再自动退到 400kHz 安全频率。
  // 每次重试前都 end 一次，避免上一次半初始化状态残留。
  const uint32_t freqs[] = { SD_MMC_MAIN_FREQ_KHZ, SD_MMC_SAFE_FREQ_KHZ };
  const char* labels[] = { "default", "safe" };

  for (int f = 0; f < 2; f++) {
    for (int attempt = 1; attempt <= SD_INIT_RETRY_COUNT; attempt++) {
      Serial.print("[SD] init attempt ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.print(SD_INIT_RETRY_COUNT);
      Serial.print(", ");
      Serial.println(labels[f]);

      SD_MMC.end();
      delay(SD_REINIT_DELAY_MS);

      if (initSDCardOnce(freqs[f], labels[f])) {
        return true;
      }
    }
  }

  Serial.println("[SD] all init attempts failed");
  return false;
}


// ===================== 9.1 SD 状态检查与重试工具 =====================
void printSDStatus(const char* prefix) {
  Serial.print(prefix);
  Serial.print(" sdOK=");
  Serial.println(sdOK ? "true" : "false");

  if (sdOK) {
    Serial.print(prefix);
    Serial.print(" cardSizeMB=");
    Serial.println((uint32_t)(SD_MMC.cardSize() / (1024 * 1024)));

    Serial.print(prefix);
    Serial.print(" usedBytesMB=");
    Serial.println((uint32_t)(SD_MMC.usedBytes() / (1024 * 1024)));
  }
}

bool ensureSDReady() {
  // 方案1：事件处理和拍照前主动确认 SD 是否仍可用，避免拍照后才发现 sdOK=false。
  if (sdOK && SD_MMC.cardType() != CARD_NONE) {
    return true;
  }

  Serial.println("[SD] SD not ready, try reinit SD before capture/file operation...");
  SD_MMC.end();
  delay(500);
  sdOK = initSDCard();
  printSDStatus("[SD]");
  return sdOK;
}

bool saveFrameToSD(camera_fb_t* fb, const String& path) {
  if (!fb) {
    Serial.println("[PHOTO] save failed: fb is null");
    return false;
  }

  if (!ensureSDReady()) {
    Serial.println("[PHOTO] SD still not ok after reinit, skip local save");
    return false;
  }

  if (!SD_MMC.exists(PHOTO_DIR)) {
    if (!SD_MMC.mkdir(PHOTO_DIR)) {
      Serial.println("[PHOTO] create /photos failed");
    }
  }

  File file = SD_MMC.open(path.c_str(), FILE_WRITE);

  if (!file) {
    Serial.println("[PHOTO] first SD open failed, reinit and retry...");
    SD_MMC.end();
    delay(500);
    sdOK = initSDCard();

    if (sdOK) {
      file = SD_MMC.open(path.c_str(), FILE_WRITE);
    }
  }

  if (!file) {
    Serial.println("[PHOTO] SD open failed after retry");
    sdOK = false;
    return false;
  }

  size_t written = file.write(fb->buf, fb->len);
  file.flush();
  file.close();

  Serial.print("[PHOTO] SD write bytes=");
  Serial.print(written);
  Serial.print("/");
  Serial.println(fb->len);

  if (written != fb->len) {
    Serial.println("[PHOTO] SD write incomplete");
    sdOK = false;
    return false;
  }

  if (!SD_MMC.exists(path.c_str())) {
    Serial.println("[PHOTO] file not found after write");
    sdOK = false;
    return false;
  }

  Serial.print("[PHOTO] saved to SD: ");
  Serial.println(path);
  return true;
}

// ===================== 10. 本地记录与 WebServer =====================
void appendRecord(const String& eventType, const String& meta, const PhotoResult& photo) {
  if (!ensureSDReady()) {
    Serial.println("[RECORD] SD not ready, skip record append");
    return;
  }

  String lines[MAX_RECORDS];
  int count = 0;

  if (SD_MMC.exists(RECORDS_FILE)) {
    File rf = SD_MMC.open(RECORDS_FILE, FILE_READ);
    if (rf) {
      bool firstLine = true;
      while (rf.available()) {
        String line = rf.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (firstLine && line.startsWith("event,")) {
          firstLine = false;
          continue;
        }
        firstLine = false;

        if (count < MAX_RECORDS) {
          lines[count++] = line;
        } else {
          for (int i = 1; i < MAX_RECORDS; i++) {
            lines[i - 1] = lines[i];
          }
          lines[MAX_RECORDS - 1] = line;
        }
      }
      rf.close();
    }
  }

  String newLine = csvSafe(eventType) + "," + makeSimpleTime() + "," + csvSafe(meta) + "," +
                   csvSafe(photo.sdPath) + "," + csvSafe(photo.imageUrl) + "," + uploadStatusText(photo.uploaded);

  if (count < MAX_RECORDS) {
    lines[count++] = newLine;
  } else {
    for (int i = 1; i < MAX_RECORDS; i++) {
      lines[i - 1] = lines[i];
    }
    lines[MAX_RECORDS - 1] = newLine;
  }

  File wf = SD_MMC.open(RECORDS_FILE, FILE_WRITE);
  if (!wf) {
    Serial.println("[RECORD] open records failed");
    return;
  }

  wf.println("event,time,meta,local_path,cloud_url,upload_status");
  for (int i = 0; i < count; i++) {
    wf.println(lines[i]);
  }
  wf.close();

  Serial.print("[RECORD] appended, count=");
  Serial.println(count);
}

int splitCsvLine(const String& line, String fields[], int maxFields) {
  int count = 0;
  int start = 0;
  for (int i = 0; i <= line.length() && count < maxFields; i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      fields[count++] = line.substring(start, i);
      start = i + 1;
    }
  }
  return count;
}

String htmlHeader(const String& title) {
  String html = "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += String("<title>") + htmlEscape(title) + "</title>";
  html += "<style>body{font-family:Arial,Helvetica,sans-serif;margin:18px;background:#f6f7fb;color:#222;}";
  html += "h2{margin:8px 0 16px;}a{color:#075edb;text-decoration:none;}a:hover{text-decoration:underline;}";
  html += ".card{background:white;border:1px solid #ddd;border-radius:10px;padding:14px;margin:10px 0;box-shadow:0 1px 3px rgba(0,0,0,.05);}";
  html += "table{border-collapse:collapse;width:100%;background:white;font-size:14px;}th,td{border:1px solid #ddd;padding:7px;text-align:left;word-break:break-all;}th{background:#eef3ff;}";
  html += ".btn{display:inline-block;background:#075edb;color:white;padding:8px 12px;border-radius:6px;margin:4px 6px 4px 0;}";
  html += ".muted{color:#666;font-size:13px;}img{max-width:100%;height:auto;border-radius:8px;}";
  html += "</style></head><body>";
  return html;
}

String htmlFooter() {
  return "<p class='muted'>ESP32-CAM Package Monitor</p></body></html>";
}

void handleHome() {
  String html = htmlHeader("包裹投递监控");
  html += "<h2>包裹投递监控系统</h2>";
  html += "<div class='card'>";
  html += "<p>设备状态：在线</p>";
  html += String("<p>WiFi IP：") + WiFi.localIP().toString() + "</p>";
  html += String("<p>Camera：") + String(cameraOK ? "正常" : "异常") + "</p>";
  html += String("<p>SD卡：") + String(sdOK ? "正常" : "异常") + "</p>";
  html += String("<p>巴法云图片主题：") + htmlEscape(String(BEMFA_IMAGE_TOPIC)) + "</p>";
  html += "<a class='btn' href='/records'>查看历史记录</a>";
  html += "<a class='btn' href='/photos'>查看图片列表</a>";
  html += "<a class='btn' href='/'>刷新页面</a>";
  html += "</div>";
  html += "<div class='card'><p>说明：远程查看使用微信通知中的巴法云图云链接；本页面用于同一局域网内查看 SD 卡本地备份。</p>";
  html += String("<p>当前仅保留最近 ") + String(MAX_RECORDS) + " 条事件记录。</p></div>";
  html += htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handleRecords() {
  String html = htmlHeader("历史记录");
  html += "<h2>历史记录</h2><p><a href='/'>返回首页</a> | <a href='/records'>刷新</a></p>";

  if (!ensureSDReady() || !SD_MMC.exists(RECORDS_FILE)) {
    html += String("<div class='card'>暂无记录或 SD 卡不可用。</div>") + htmlFooter();
    server.send(200, "text/html; charset=utf-8", html);
    return;
  }

  File f = SD_MMC.open(RECORDS_FILE, FILE_READ);
  if (!f) {
    html += String("<div class='card'>无法打开 records.csv。</div>") + htmlFooter();
    server.send(500, "text/html; charset=utf-8", html);
    return;
  }

  html += "<table><tr><th>事件</th><th>时间/ms</th><th>检测数据</th><th>本地图片</th><th>云端图片</th><th>状态</th></tr>";
  bool firstLine = true;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (firstLine && line.startsWith("event,")) { firstLine = false; continue; }
    firstLine = false;

    String fields[6];
    int n = splitCsvLine(line, fields, 6);
    if (n < 6) continue;

    html += "<tr>";
    html += String("<td>") + htmlEscape(fields[0]) + "</td>";
    html += String("<td>") + htmlEscape(fields[1]) + "</td>";
    html += String("<td>") + htmlEscape(fields[2]) + "</td>";
    if (fields[3].length() > 0) {
      html += String("<td><a href='/photo?name=") + fields[3] + "'>查看</a><br><span class='muted'>" + htmlEscape(fields[3]) + "</span></td>";
    } else {
      html += "<td>无</td>";
    }
    if (fields[4].length() > 0) {
      html += String("<td><a target='_blank' href='") + fields[4] + "'>打开</a></td>";
    } else {
      html += "<td>无</td>";
    }
    html += String("<td>") + htmlEscape(fields[5]) + "</td>";
    html += "</tr>";
  }
  f.close();
  html += String("</table>") + htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handlePhotos() {
  String html = htmlHeader("图片列表");
  html += "<h2>SD 卡图片列表</h2><p><a href='/'>返回首页</a> | <a href='/photos'>刷新</a></p>";

  if (!ensureSDReady() || !SD_MMC.exists(PHOTO_DIR)) {
    html += String("<div class='card'>暂无图片或 SD 卡不可用。</div>") + htmlFooter();
    server.send(200, "text/html; charset=utf-8", html);
    return;
  }

  File root = SD_MMC.open(PHOTO_DIR);
  if (!root || !root.isDirectory()) {
    html += String("<div class='card'>无法打开 /photos 目录。</div>") + htmlFooter();
    server.send(500, "text/html; charset=utf-8", html);
    return;
  }

  html += "<table><tr><th>文件名</th><th>大小</th><th>操作</th></tr>";
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = String(file.name());
      String fullPath = name.startsWith("/") ? name : String(PHOTO_DIR) + "/" + name;
      if (fullPath.endsWith(".jpg")) {
        html += String("<tr><td>") + htmlEscape(fullPath) + "</td><td>" + String(file.size()) + " B</td>";
        html += String("<td><a href='/photo?name=") + fullPath + "'>查看</a></td></tr>";
      }
    }
    file = root.openNextFile();
  }
  root.close();

  html += String("</table>") + htmlFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void handlePhoto() {
  if (!server.hasArg("name")) {
    server.send(400, "text/plain; charset=utf-8", "missing name");
    return;
  }

  String path = server.arg("name");
  if (!safePhotoPath(path)) {
    server.send(403, "text/plain; charset=utf-8", "invalid path");
    return;
  }

  if (!ensureSDReady() || !SD_MMC.exists(path)) {
    server.send(404, "text/plain; charset=utf-8", "photo not found");
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    server.send(500, "text/plain; charset=utf-8", "open photo failed");
    return;
  }

  server.streamFile(file, "image/jpeg");
  file.close();
}

void handleNotFound() {
  server.send(404, "text/plain; charset=utf-8", "404 Not Found");
}

void startWebServer() {
  server.on("/", handleHome);
  server.on("/records", handleRecords);
  server.on("/photos", handlePhotos);
  server.on("/photo", handlePhoto);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.print("[WEB] local url: http://");
  Serial.println(WiFi.localIP());
}

// ===================== 11. 巴法云图片上传：二进制接口 =====================
bool uploadPhotoToBemfa(camera_fb_t* fb, const String& wechatMsg, String& imageUrl, int& httpCode, String& response) {
  imageUrl = "";
  httpCode = -1;
  response = "";

  if (!fb) return false;

  if (!ensureWiFi()) {
    Serial.println("[BEMFA] image upload failed: WiFi not connected");
    return false;
  }

  HTTPClient http;
  http.setTimeout(30000);

  if (!http.begin(BEMFA_IMAGE_UPLOAD_URL)) {
    Serial.println("[BEMFA] http begin failed: image upload");
    return false;
  }

  http.addHeader("Content-Type", "image/jpg");
  http.addHeader("Authorization", BEMFA_UID);
  http.addHeader("Authtopic", BEMFA_IMAGE_TOPIC);

  // 这里不使用 wechatmsg，避免上传接口和通知接口重复推送。
  // 如果你想让上传图片时自动推送微信，可以取消下一行注释：
  // http.addHeader("wechatmsg", wechatMsg);

  Serial.print("[BEMFA] uploading image, len=");
  Serial.println(fb->len);

  httpCode = http.POST(fb->buf, fb->len);
  response = http.getString();

  Serial.print("[BEMFA] image HTTP code=");
  Serial.println(httpCode);
  Serial.print("[BEMFA] image response=");
  Serial.println(response);

  http.end();

  if (httpCode > 0 && httpCode < 400) {
    imageUrl = extractImageUrl(response);
    return imageUrl.length() > 0;
  }

  return false;
}

// ===================== 11. 拍照、存卡、上传 =====================
PhotoResult captureSaveUploadPhoto(const String& tag, const String& meta) {
  PhotoResult r;
  r.captured = false;
  r.savedToSD = false;
  r.uploaded = false;
  r.sdPath = "";
  r.imageUrl = "";
  r.imageLength = 0;
  r.httpCode = -1;
  r.serverResponse = "";

  if (!cameraOK) {
    Serial.println("[PHOTO] camera not ok");
    return r;
  }

  // 方案1：拍照前先确认 SD 状态。即使 SD 不可用，也继续拍照并上传巴法云，
  // 但会在日志中明确标记本地保存失败。
  bool sdReadyBeforeCapture = ensureSDReady();
  Serial.print("[PHOTO] sdReadyBeforeCapture=");
  Serial.println(sdReadyBeforeCapture ? "true" : "false");

  flashOn();
  delay(FLASH_PRELIGHT_MS);

  camera_fb_t* fb = esp_camera_fb_get();

  flashOff();

  if (!fb) {
    Serial.println("[PHOTO] capture failed");
    return r;
  }

  r.captured = true;
  r.imageLength = fb->len;

  r.sdPath = String(PHOTO_DIR) + "/" + tag + "_" + String(millis()) + ".jpg";

  r.savedToSD = saveFrameToSD(fb, r.sdPath);
  if (!r.savedToSD) {
    r.sdPath = "";
  }

  String uploadMsg = tag + " " + meta;
  r.uploaded = uploadPhotoToBemfa(fb, uploadMsg, r.imageUrl, r.httpCode, r.serverResponse);

  esp_camera_fb_return(fb);

  return r;
}

// ===================== 12. 事件处理 =====================
void handleDoorbell(const String& cmd) {
  String msg = "【门铃触发】有人靠近门口，系统已进入预检测流程。";
  bool ok = sendBemfaWechat(msg);
  Serial.println(ok ? "ACK:DOORBELL" : "ERR:DOORBELL_NOTIFY_FAILED");
}

void handleFalseTrigger(const String& cmd) {
  String meta = getMetaFromCommand(cmd);
  String msg = "【误触发】PIR 检测到人体活动，但 FSR402 与超声波投票未确认包裹。";
  if (meta.length() > 0) {
    msg += String("\n") + meta;
  }

  bool ok = sendBemfaWechat(msg);
  Serial.println(ok ? "ACK:FALSE_TRIGGER" : "ERR:FALSE_TRIGGER_NOTIFY_FAILED");
}

void handleArrived(const String& cmd) {
  Serial.println("[EVENT] ARRIVED start, ensure SD before capture");
  ensureSDReady();

  String meta = getMetaFromCommand(cmd);
  String baseMsg = "【包裹已送达】系统已确认包裹放置。";
  if (meta.length() > 0) {
    baseMsg += String("\n") + meta;
  }

  PhotoResult photo = captureSaveUploadPhoto("ARRIVED", meta);
  appendRecord("ARRIVED", meta, photo);

  String msg = baseMsg;
  msg += String("\n照片长度=") + String(photo.imageLength);
  msg += photo.savedToSD ? (String("\nSD卡保存成功：") + photo.sdPath) : String("\nSD卡保存失败或未插卡");
  if (photo.savedToSD && WiFi.status() == WL_CONNECTED) {
    msg += String("\n局域网查看：http://") + WiFi.localIP().toString() + "/records";
  }

  if (photo.uploaded) {
    msg += "\n图片已上传巴法云图云。";
    msg += String("\n") + photo.imageUrl;
  } else {
    msg += String("\n图片上传失败，HTTP=") + String(photo.httpCode);
  }

  bool notifyOK = sendBemfaWechat(msg, photo.imageUrl);

  if (photo.captured && notifyOK) {
    Serial.println("ACK:ARRIVED");
  } else {
    Serial.println("ERR:ARRIVED_FAILED");
  }
}

void handleRemoved(const String& cmd) {
  Serial.println("[EVENT] REMOVED start, ensure SD before capture");
  ensureSDReady();

  String meta = getMetaFromCommand(cmd);
  String baseMsg = "【包裹已取走】系统已确认包裹离开。";
  if (meta.length() > 0) {
    baseMsg += String("\n") + meta;
  }

  PhotoResult photo = captureSaveUploadPhoto("REMOVED", meta);
  appendRecord("REMOVED", meta, photo);

  String msg = baseMsg;
  msg += String("\n照片长度=") + String(photo.imageLength);
  msg += photo.savedToSD ? (String("\nSD卡保存成功：") + photo.sdPath) : String("\nSD卡保存失败或未插卡");
  if (photo.savedToSD && WiFi.status() == WL_CONNECTED) {
    msg += String("\n局域网查看：http://") + WiFi.localIP().toString() + "/records";
  }

  if (photo.uploaded) {
    msg += "\n图片已上传巴法云图云。";
    msg += String("\n") + photo.imageUrl;
  } else {
    msg += String("\n图片上传失败，HTTP=") + String(photo.httpCode);
  }

  bool notifyOK = sendBemfaWechat(msg, photo.imageUrl);

  if (photo.captured && notifyOK) {
    Serial.println("ACK:REMOVED");
  } else {
    Serial.println("ERR:REMOVED_FAILED");
  }
}

void handleShutdown() {
  // 调试阶段 ESP32-CAM 常供电，这里不真正休眠，只回复主控即可。
  Serial.println("CAM_SLEEP_OK");
}

void handlePing() {
  Serial.print("[STATUS] cameraOK=");
  Serial.println(cameraOK ? "true" : "false");
  printSDStatus("[STATUS]");
  Serial.println("ACK:PING");
}

void handleSDStatus() {
  Serial.println("[SDTEST] manual SD reinit start");
  SD_MMC.end();
  delay(500);
  sdOK = initSDCard();
  printSDStatus("[SDTEST]");
  Serial.println("ACK:SDSTATUS");
}

void runStandaloneTest() {
  Serial.println("[TEST] standalone Bemfa test start");
  sendBemfaWechat("【测试】ESP32-CAM 已连接 WiFi，巴法云微信通知测试。准备拍照上传。 ");

  PhotoResult photo = captureSaveUploadPhoto("TEST", "BOOT_TEST");

  String msg = "【测试】ESP32-CAM 拍照上传测试完成。";
  msg += String("\n照片长度=") + String(photo.imageLength);
  msg += photo.savedToSD ? (String("\nSD卡保存成功：") + photo.sdPath) : String("\nSD卡保存失败或未插卡");
  if (photo.savedToSD && WiFi.status() == WL_CONNECTED) {
    msg += String("\n局域网查看：http://") + WiFi.localIP().toString() + "/records";
  }
  msg += photo.uploaded ? (String("\n图片上传成功：") + photo.imageUrl) : (String("\n图片上传失败，HTTP=") + String(photo.httpCode));

  sendBemfaWechat(msg, photo.imageUrl);
  Serial.println("[TEST] standalone Bemfa test end");
}

// ===================== 13. 初始化 =====================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(1000);

  Serial.println();
  Serial.println("========== ESP32-CAM Bemfa Package Monitor ==========");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // 先初始化 SD，再初始化摄像头，降低上电瞬间外设同时启动造成的失败概率。
  sdOK = initSDCard();
  cameraOK = initCamera();

  connectWiFi();
  startWebServer();

  Serial.print("[CONFIG] image topic=");
  Serial.println(BEMFA_IMAGE_TOPIC);
  Serial.print("[CONFIG] cameraOK=");
  Serial.println(cameraOK ? "true" : "false");
  Serial.print("[CONFIG] sdOK=");
  Serial.println(sdOK ? "true" : "false");

#if RUN_STANDALONE_TEST_ON_BOOT
  runStandaloneTest();
#endif

  delay(500);
  Serial.println("CAM_READY");
}

// ===================== 14. 主循环：等待 ESP32 主控 UART 命令 =====================
void loop() {
  server.handleClient();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) {
      return;
    }

    Serial.print("[CMD] ");
    Serial.println(cmd);

    if (cmd == "CMD:PING") {
      handlePing();
    } else if (cmd == "CMD:SDSTATUS") {
      handleSDStatus();
    } else if (cmd == "EVENT:DOORBELL") {
      handleDoorbell(cmd);
    } else if (cmd.startsWith("EVENT:FALSE_TRIGGER")) {
      handleFalseTrigger(cmd);
    } else if (cmd.startsWith("EVENT:ARRIVED")) {
      handleArrived(cmd);
    } else if (cmd.startsWith("EVENT:REMOVED")) {
      handleRemoved(cmd);
    } else if (cmd == "CMD:SHUTDOWN") {
      handleShutdown();
    } else {
      Serial.println("ERR:UNKNOWN_CMD");
    }
  }

  delay(20);
}
