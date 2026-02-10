#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "INA226.h"
#include <TinyGPS++.h>
#include <RTClib.h>

// WiFi
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// PIN PLAN (ESP32-S3)
// I2C for INA226 + DS3231
#define I2C_SDA 1
#define I2C_SCL 2

// INA226 addresses
#define INA_IN_ADDR 0x40
#define INA_OUT_ADDR 0x41

// GPS UART1
#define GPS_RX_PIN 39  // connect to GPS TX
#define GPS_TX_PIN 40  // connect to GPS RX

// SD SPI
#define SD_CS 34
#define SD_MOSI 35
#define SD_SCK 36
#define SD_MISO 37

// OV2640 Camera pins
#define CAM_PWDN 42
#define CAM_RESET 41
#define CAM_XCLK 16
#define CAM_SIOD 12
#define CAM_SIOC 13

#define CAM_D0 4
#define CAM_D1 5
#define CAM_D2 6
#define CAM_D3 7
#define CAM_D4 8
#define CAM_D5 9
#define CAM_D6 10
#define CAM_D7 11

#define CAM_VSYNC 18
#define CAM_HREF 21
#define CAM_PCLK 17

// Objects
WebServer server(80);
INA226 ina_in(INA_IN_ADDR);
INA226 ina_out(INA_OUT_ADDR);
TinyGPSPlus gps;
RTC_DS3231 rtc;
bool rtcOK = false;

// Data Structure
struct MeasurementData {
  char timestamp[25];
  float vin, iin, pin;
  float vout, iout, pout;
  float efficiency;
  double lat, lng;
};

QueueHandle_t dataQueue;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t camMutex;

volatile bool capturePhoto = false;
volatile uint32_t photoCounter = 0;

// Save the latest settings to display on the website (option)
MeasurementData latest;
SemaphoreHandle_t latestMutex;

// Helpers
static void makeTimestamp(char* out, size_t outSize) {
  if (rtcOK) {
    DateTime now = rtc.now();
    snprintf(out, outSize, "%04d-%02d-%02dT%02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  } else {
    // Fallback by millis
    uint32_t ms = millis();
    snprintf(out, outSize, "ms_%lu", (unsigned long)ms);
  }
}

// Camera init (OV2640 on ESP32-S3)
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = CAM_D0;
  config.pin_d1 = CAM_D1;
  config.pin_d2 = CAM_D2;
  config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4;
  config.pin_d5 = CAM_D5;
  config.pin_d6 = CAM_D6;
  config.pin_d7 = CAM_D7;

  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;

  config.pin_sscb_sda = CAM_SIOD;
  config.pin_sscb_scl = CAM_SIOC;

  config.pin_pwdn = CAM_PWDN;
  config.pin_reset = CAM_RESET;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 12;           // 10-14
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 14;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  Serial.println("Camera initialized");
  return true;
}

// SD init (SPI)
bool initSD_SPI() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // Reduce risk: start with 20-25MHz
  if (!SD.begin(SD_CS, SPI, 25000000)) {
    Serial.println("SD(SPI) mount failed");
    return false;
  }
  Serial.println("SD(SPI) mounted");

  if (!SD.exists("/photos")) {
    SD.mkdir("/photos");
    Serial.println("Created /photos");
  }

  if (!SD.exists("/datalog.csv")) {
    File f = SD.open("/datalog.csv", FILE_WRITE);
    if (f) {
      f.println("Timestamp,Vin(V),Iin(A),Pin(W),Vout(V),Iout(A),Pout(W),Efficiency(%),Latitude,Longitude");
      f.close();
      Serial.println("Created /datalog.csv header");
    }
  }
  return true;
}

// Web handlers
void handleRoot() {
  MeasurementData snap{};
  if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = latest;
    xSemaphoreGive(latestMutex);
  }

  String html;
  html += "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:16px;} .box{max-width:900px;margin:auto;} img{max-width:100%;height:auto;border:1px solid #ccc;} ";
  html += "button{padding:10px 16px;font-size:16px;margin:6px;} pre{background:#f6f6f6;padding:10px;overflow:auto;}</style>";
  html += "</head><body><div class='box'>";
  html += "<h2>ESP32-S3 OV2640 + INA226 Logger</h2>";
  html += "<img id='img' src='/jpg?t=0' />";
  html += "<div><button onclick='cap()'>Capture Photo</button>";
  html += "<button onclick='snap()'>Refresh Image</button></div>";

  html += "<pre id='st'>";
  html += "Time: " + String(snap.timestamp) + "\n";
  html += "Vin=" + String(snap.vin, 3) + " V  Iin=" + String(snap.iin, 6) + " A  Pin=" + String(snap.pin, 6) + " W\n";
  html += "Vout=" + String(snap.vout, 3) + " V  Iout=" + String(snap.iout, 6) + " A  Pout=" + String(snap.pout, 6) + " W\n";
  html += "Eff=" + String(snap.efficiency, 2) + " %\n";
  html += "Lat=" + String(snap.lat, 6) + "  Lng=" + String(snap.lng, 6) + "\n";
  html += "Photos saved: " + String(photoCounter) + "\n";
  html += "</pre>";

  html += "<script>";
  html += "function snap(){document.getElementById('img').src='/jpg?t='+Date.now();}";
  html += "function cap(){fetch('/capture').then(()=>setTimeout(()=>{snap();},400));}";
  html += "setInterval(()=>{fetch('/status').then(r=>r.text()).then(t=>{document.getElementById('st').textContent=t;});}, 2000);";
  html += "</script>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// Send JPEG 1 Frame
void handleJPG() {
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
}

void handleCapture() {
  capturePhoto = true;
  server.send(200, "text/plain", "OK");
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

// Task: Data Collector
void taskDataCollector(void* pv) {
  (void)pv;
  Serial.println("Task DataCollector: Running");

  for (;;) {
    MeasurementData d{};
    makeTimestamp(d.timestamp, sizeof(d.timestamp));

    // INA226
    d.vin = ina_in.getBusVoltage();
    d.iin = ina_in.getCurrent();
    d.pin = ina_in.getPower();

    d.vout = ina_out.getBusVoltage();
    d.iout = ina_out.getCurrent();
    d.pout = ina_out.getPower();

    d.efficiency = (d.pin > 0.000001f) ? (d.pout / d.pin) * 100.0f : 0.0f;

    // GPS
    while (Serial1.available()) gps.encode(Serial1.read());
    d.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    d.lng = gps.location.isValid() ? gps.location.lng() : 0.0;

    // Update latest
    if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      latest = d;
      xSemaphoreGive(latestMutex);
    }

    xQueueSend(dataQueue, &d, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task: SD Logger (CSV)
void taskSDLogger(void* pv) {
  (void)pv;
  Serial.println("Task SDLogger: Running");

  for (;;) {
    MeasurementData d{};
    if (xQueueReceive(dataQueue, &d, portMAX_DELAY) == pdPASS) {
      if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        File f = SD.open("/datalog.csv", FILE_APPEND);
        if (f) {
          char line[256];
          snprintf(line, sizeof(line),
                   "%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f",
                   d.timestamp,
                   d.vin, d.iin, d.pin,
                   d.vout, d.iout, d.pout,
                   d.efficiency,
                   d.lat, d.lng);
          f.println(line);
          f.close();
          Serial.print("Logged: ");
          Serial.println(line);
        } else {
          Serial.println("SDLogger: open datalog.csv failed");
        }
        xSemaphoreGive(sdMutex);
      } else {
        Serial.println("SDLogger: sdMutex timeout");
      }
    }
  }
}

// Task: Camera capture to SD (/capture)
void taskCamera(void* pv) {
  (void)pv;
  Serial.println("Task Camera: Running");

  for (;;) {
    if (!capturePhoto) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    capturePhoto = false;

    // lock camera
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
      Serial.println("CameraTask: camMutex timeout");
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("CameraTask: capture failed");
      xSemaphoreGive(camMutex);
      continue;
    }

    // lock SD
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      char ts[25];
      makeTimestamp(ts, sizeof(ts));

      // Use timestamp
      for (int i = 0; ts[i]; i++)
        if (ts[i] == ':') ts[i] = '_';

      char filename[80];
      snprintf(filename, sizeof(filename), "/photos/img_%s.jpg", ts);

      File file = SD.open(filename, FILE_WRITE);
      if (file) {
        file.write(fb->buf, fb->len);
        file.close();
        photoCounter++;
        Serial.printf("Photo saved: %s (%u bytes)\n", filename, (unsigned)fb->len);
      } else {
        Serial.println("CameraTask: open file failed");
      }

      xSemaphoreGive(sdMutex);
    } else {
      Serial.println("CameraTask: sdMutex timeout");
    }

    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);
  }
}

// Setup / Loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- ESP32-S3 OV2640 + INA226 + SD(SPI) + Web ---");

  // RTOS objects
  sdMutex = xSemaphoreCreateMutex();
  camMutex = xSemaphoreCreateMutex();
  latestMutex = xSemaphoreCreateMutex();
  dataQueue = xQueueCreate(10, sizeof(MeasurementData));
  if (!sdMutex || !camMutex || !latestMutex || !dataQueue) {
    Serial.println("Failed to create RTOS objects");
    while (1)
      ;
  }

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // INA226 init
  if (!ina_in.begin() || !ina_out.begin()) {
    Serial.println("INA226 not found (check wiring/address)");
    while (1)
      ;
  }

  // Calibration for low-current work (Rshunt = 0.1 ohm)
  ina_in.setMaxCurrentShunt(0.2, 0.1);   // 200mA, 0.1Ω
  ina_out.setMaxCurrentShunt(0.2, 0.1);  // 200mA, 0.1Ω
  Serial.println("INA226 OK");

  // RTC
  rtcOK = rtc.begin();
  if (rtcOK) Serial.println("RTC OK");
  else Serial.println("RTC not found (timestamp fallback = millis)");

  // GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART1 started");

  // SD
  if (!initSD_SPI())
    while (1)
      ;

  // Camera
  if (!initCamera())
    while (1)
      ;

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP = ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed (server still starts, but not reachable)");
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/jpg", handleJPG);
  server.on("/capture", handleCapture);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("WebServer started. Open http://<ip>/");

  // Tasks (S3 dual-core)
  xTaskCreatePinnedToCore(taskDataCollector, "DataCollector", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSDLogger, "SDLogger", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskCamera, "CameraTask", 8192, NULL, 1, NULL, 1);

  Serial.println("Setup complete.");
}

void loop() {
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(10));
}
