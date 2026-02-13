#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>

#include "SensorManager.h"
#include "CameraManager.h"
#include "WifiManager.h"

// FEATURE SWITCHES
#define ENABLE_CAMERA  1
#define ENABLE_SD      0
#define ENABLE_SENSORS 1
#define ENABLE_WIFI    1

// WiFi
const char* ssid = "ESP32S3-CAM";
const char* password = "12345678";

// SD_MMC 1-bit
#define SD_MMC_CMD 38
#define SD_MMC_CLK 39
#define SD_MMC_D0  40

QueueHandle_t dataQueue;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t camMutex;
SemaphoreHandle_t latestMutex;

volatile bool capturePhoto = false;
volatile uint32_t photoCounter = 0;
MeasurementData latest;

// Objects
SensorManager sensors;
CameraManager cam;
WifiManager wifi;

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

// Tasks
void taskDataCollector(void* pv) {
  (void)pv;
  for (;;) {
    #if ENABLE_SENSORS
        MeasurementData d = sensors.readData();
    #else
        MeasurementData d{};
    #endif

    if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      latest = d;
      xSemaphoreGive(latestMutex);
    }

    #if ENABLE_SD && ENABLE_SENSORS
        xQueueSend(dataQueue, &d, pdMS_TO_TICKS(10));
    #endif
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskSDLogger(void* pv) {
  (void)pv;
#if !(ENABLE_SD && ENABLE_SENSORS)
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
    
    // Use CameraManager
    camera_fb_t* fb = cam.capture();
    if (!fb) { xSemaphoreGive(camMutex); continue; }

    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
      char ts[32];
      sensors.makeTimestamp(ts, sizeof(ts));
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

    cam.returnFrame(fb);
    xSemaphoreGive(camMutex);
  }
#endif
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- ESP32-S3 CAM (Fully Object Oriented) ---");

  sdMutex = xSemaphoreCreateMutex();
  camMutex = xSemaphoreCreateMutex();
  latestMutex = xSemaphoreCreateMutex();
  dataQueue = xQueueCreate(10, sizeof(MeasurementData));

#if ENABLE_SENSORS
  sensors.begin();
#endif
  
#if ENABLE_CAMERA
  if (!cam.begin()) {
    Serial.println("Camera init failed");
  } else {
    Serial.println("Camera initialized");
  }
#endif

#if ENABLE_SD
  if (!initSDMMC_1bit()) {
    Serial.println("SD init failed");
  }
#endif

#if ENABLE_WIFI
  wifi.begin(ssid, password, &sensors, &cam, camMutex, latestMutex, &latest, &capturePhoto, &photoCounter);
#endif

  xTaskCreatePinnedToCore(taskDataCollector, "DataCollector", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSDLogger,      "SDLogger",      4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskCamera,        "CameraTask",    8192, NULL, 1, NULL, 1);
}

void loop() {
#if ENABLE_WIFI
  wifi.handleClient();
#endif
  vTaskDelay(pdMS_TO_TICKS(10));
}
