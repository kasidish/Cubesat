#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <FS.h>
#include <SD_MMC.h>

#include "INA226.h"
#include <TinyGPS++.h>
#include <RTClib.h>

// FEATURE SWITCHES MODE 0 = OFF UNABLE MODE 1 = ON ENABLE
#define ENABLE_WIFI    1
#define ENABLE_CAMERA  1
#define ENABLE_SD      0
#define ENABLE_INA226  0
#define ENABLE_RTC     0
#define ENABLE_GPS     0

// WiFi
const char* ssid = "ESP32S3-CAM";
const char* password = "12345678";

// I2C (INA226 + RTC)
#define I2C_SDA 1
#define I2C_SCL 2

#define INA_IN_ADDR  0x40
#define INA_OUT_ADDR 0x41
INA226 ina_in(INA_IN_ADDR);
INA226 ina_out(INA_OUT_ADDR);

RTC_DS3231 rtc;
bool rtcOK = false;
bool inaOK = false;

// GPS (UART)
static const int GPS_RX_PIN = 21;  // GPS TX -> ESP RX
static const int GPS_TX_PIN = 47;  // GPS RX <- ESP TX
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// SD_MMC 1-bit
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0  40

// Camera pins
#define CAM_SIOD 4
#define CAM_SIOC 5
#define CAM_VSYNC 6
#define CAM_HREF  7
#define CAM_PCLK  13
#define CAM_XCLK  15

#define CAM_D0 11  // Y2
#define CAM_D1 9   // Y3
#define CAM_D2 8   // Y4
#define CAM_D3 10  // Y5
#define CAM_D4 12  // Y6
#define CAM_D5 18  // Y7
#define CAM_D6 17  // Y8
#define CAM_D7 16  // Y9

#define CAM_PWDN  -1
#define CAM_RESET -1

// Web/RTOS
WebServer server(80);

struct MeasurementData {
  char timestamp[32];
  float vin, iin, pin;
  float vout, iout, pout;
  float efficiency;
  double lat, lng;
};

QueueHandle_t dataQueue;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t camMutex;
SemaphoreHandle_t latestMutex;

volatile bool capturePhoto = false;
volatile uint32_t photoCounter = 0;
MeasurementData latest;

// Timestamp
static void makeTimestamp(char* out, size_t outSize) {
#if ENABLE_RTC
  if (rtcOK) {
    DateTime now = rtc.now();
    snprintf(out, outSize, "%04d-%02d-%02dT%02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return;
  }
#endif
  snprintf(out, outSize, "ms_%lu", (unsigned long)millis());
}

// SDMMC_1bit (SD card)
bool initSDMMC_1bit() {
#if !ENABLE_SD
  return true;
#else
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD_MMC mount failed");
    return false;
  }
  Serial.println("SD_MMC mounted");

  if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");

  if (!SD_MMC.exists("/datalog.csv")) {
    File f = SD_MMC.open("/datalog.csv", FILE_WRITE);
    if (f) {
      f.println("Timestamp,Vin(V),Iin(A),Pin(W),Vout(V),Iout(A),Pout(W),Efficiency(%),Latitude,Longitude");
      f.close();
    }
  }
  return true;
#endif
}

// init camare setting configuration
bool initCamera() {
#if !ENABLE_CAMERA
  return true;
#else
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = CAM_D0; config.pin_d1 = CAM_D1; config.pin_d2 = CAM_D2; config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4; config.pin_d5 = CAM_D5; config.pin_d6 = CAM_D6; config.pin_d7 = CAM_D7;

  config.pin_xclk  = CAM_XCLK;
  config.pin_pclk  = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href  = CAM_HREF;

  config.pin_sscb_sda = CAM_SIOD;
  config.pin_sscb_scl = CAM_SIOC;

  config.pin_pwdn  = CAM_PWDN;
  config.pin_reset = CAM_RESET;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA; // 640x480
    config.jpeg_quality = 12; // 10-14
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size   = FRAMESIZE_QVGA; // 320x240
    config.jpeg_quality = 14;
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) Serial.printf("Camera PID: 0x%04X\n", s->id.PID);

  Serial.println("Camera initialized");
  return true;
#endif
}

// Check Status WiFi
void printWiFiStatus() {
  wl_status_t s = WiFi.status();
  Serial.print("WiFi.status = "); Serial.println((int)s);

  // 1 = WL_NO_SSID_AVAIL
  // 3 = WL_CONNECTED
  // 4 = WL_CONNECT_FAILED
  // 5 = WL_CONNECTION_LOST
  // 6 = WL_DISCONNECTED
}


// Web
void handleRoot() {
  MeasurementData snap{};
  if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = latest;
    xSemaphoreGive(latestMutex);
  }

  String html;
  html += "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:16px;} img{max-width:100%;height:auto;border:1px solid #ccc;} button{padding:10px 16px;font-size:16px;margin:6px;} pre{background:#f6f6f6;padding:10px;}</style>";
  html += "</head><body>";
  html += "<h2>ESP32-S3 CAM Test</h2>";
  html += "<img id='img' src='/jpg?t=0' />";
  html += "<div><button onclick='cap()'>Capture Photo</button><button onclick='snap()'>Refresh</button></div>";
  html += "<pre id='st'>Loading...</pre>";
  html += "<script>";
  html += "function snap(){document.getElementById('img').src='/jpg?t='+Date.now();}";
  html += "function cap(){fetch('/capture').then(()=>setTimeout(snap,400));}";
  html += "setInterval(()=>{fetch('/status').then(r=>r.text()).then(t=>{document.getElementById('st').textContent=t;});}, 2000);";
  html += "</script>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleJPG() {
#if !ENABLE_CAMERA
  server.send(503, "text/plain", "Camera disabled");
  return;
#else
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    server.send(503, "text/plain", "Camera busy");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  WiFiClient client = server.client();
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
#endif
}

void handleCapture() {
#if !ENABLE_CAMERA || !ENABLE_SD
  server.send(503, "text/plain", "Capture disabled (need CAMERA+SD)");
#else
  capturePhoto = true;
  server.send(200, "text/plain", "OK");
#endif
}

void handleStatus() {
  MeasurementData snap{};
  if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = latest;
    xSemaphoreGive(latestMutex);
  }

  String s;
  s += String("Time: ") + snap.timestamp + "\n";
  s += "Vin=" + String(snap.vin, 3) + " V  Iin=" + String(snap.iin, 6) + " A  Pin=" + String(snap.pin, 6) + " W\n";
  s += "Vout=" + String(snap.vout, 3) + " V  Iout=" + String(snap.iout, 6) + " A  Pout=" + String(snap.pout, 6) + " W\n";
  s += "Eff=" + String(snap.efficiency, 2) + " %\n";
  s += "Lat=" + String(snap.lat, 6) + "  Lng=" + String(snap.lng, 6) + "\n";
  s += "Photos saved: " + String(photoCounter) + "\n";
  server.send(200, "text/plain", s);
}

// Tasks
void taskDataCollector(void* pv) {
  (void)pv;
  for (;;) {
    MeasurementData d{};
    makeTimestamp(d.timestamp, sizeof(d.timestamp));

#if ENABLE_INA226
    if (inaOK) {
      d.vin = ina_in.getBusVoltage();
      d.iin = ina_in.getCurrent();
      d.pin = ina_in.getPower();
      d.vout = ina_out.getBusVoltage();
      d.iout = ina_out.getCurrent();
      d.pout = ina_out.getPower();
      d.efficiency = (d.pin > 0.000001f) ? (d.pout / d.pin) * 100.0f : 0.0f;
    }
#endif

#if ENABLE_GPS
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    d.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    d.lng = gps.location.isValid() ? gps.location.lng() : 0.0;
#endif

    if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      latest = d;
      xSemaphoreGive(latestMutex);
    }

#if ENABLE_SD && ENABLE_INA226
    xQueueSend(dataQueue, &d, pdMS_TO_TICKS(10));
#endif
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskSDLogger(void* pv) {
  (void)pv;
#if !(ENABLE_SD && ENABLE_INA226)
  vTaskDelete(NULL);
#else
  for (;;) {
    MeasurementData d{};
    if (xQueueReceive(dataQueue, &d, portMAX_DELAY) == pdPASS) {
      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        File f = SD_MMC.open("/datalog.csv", FILE_APPEND);
        if (f) {
          char line[256];
          snprintf(line, sizeof(line),
                   "%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f",
                   d.timestamp, d.vin, d.iin, d.pin, d.vout, d.iout, d.pout, d.efficiency, d.lat, d.lng);
          f.println(line);
          f.close();
        }
        xSemaphoreGive(sdMutex);
      }
    }
  }
#endif
}

void taskCamera(void* pv) {
  (void)pv;
#if !(ENABLE_CAMERA && ENABLE_SD)
  vTaskDelete(NULL);
#else
  for (;;) {
    if (!capturePhoto) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    capturePhoto = false;

    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(2000)) != pdTRUE) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { xSemaphoreGive(camMutex); continue; }

    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      char ts[32];
      makeTimestamp(ts, sizeof(ts));
      for (int i = 0; ts[i]; i++) if (ts[i] == ':') ts[i] = '_';
      char filename[96];
      snprintf(filename, sizeof(filename), "/photos/img_%s.jpg", ts);

      File file = SD_MMC.open(filename, FILE_WRITE);
      if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        photoCounter++;
      }
      xSemaphoreGive(sdMutex);
    }

    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);
  }
#endif
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- ESP32-S3 CAM + SD_MMC + Web (Modular) ---");

  sdMutex = xSemaphoreCreateMutex();
  camMutex = xSemaphoreCreateMutex();
  latestMutex = xSemaphoreCreateMutex();
  dataQueue = xQueueCreate(10, sizeof(MeasurementData));

  Wire.begin(I2C_SDA, I2C_SCL);

#if ENABLE_RTC
  rtcOK = rtc.begin();
#endif

#if ENABLE_GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

#if ENABLE_INA226
  inaOK = ina_in.begin() && ina_out.begin();
  if (inaOK) {
    ina_in.setMaxCurrentShunt(0.2, 0.1);
    ina_out.setMaxCurrentShunt(0.2, 0.1);
    Serial.println("INA226 OK");
  } else {
    Serial.println("INA226 not found -> continue without INA");
  }
#endif

#if ENABLE_SD
  if (!initSDMMC_1bit()) {
    Serial.println("SD init failed");
  }
#endif

#if ENABLE_CAMERA
  if (!initCamera()) {
    Serial.println("Camera init failed");
  }
#endif

#if ENABLE_WIFI
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32S3-CAM", "12345678");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
#endif
  //http://192.168.4.1/
  server.on("/", handleRoot);
  server.on("/jpg", handleJPG);
  server.on("/capture", handleCapture);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("WebServer started");

  xTaskCreatePinnedToCore(taskDataCollector, "DataCollector", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSDLogger,      "SDLogger",      4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskCamera,        "CameraTask",    8192, NULL, 1, NULL, 1);
}

void loop() {
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(10));
}
