  #include <Wire.h>
  #include <WiFi.h>
  #include <WebServer.h>
  #include "esp_camera.h"
  #include <FS.h>
  #include <SD_MMC.h>
  #include "Wire.h"

  #include "INA226.h"
  #include <TinyGPS++.h>
  #include <RTClib.h>
  #include "esp_heap_caps.h"

  // FEATURE SWITCHES MODE 0 = OFF UNABLE MODE 1 = ON ENABLE
  #define ENABLE_WIFI    1
  #define ENABLE_CAMERA  1
  #define ENABLE_SD      1
  #define ENABLE_INA226  0
  #define ENABLE_RTC     0
  #define ENABLE_GPS     0

  // WiFi
  const char* ssid = "ESP32S3-CAM-KASIDISH";
  const char* password = "12345678";

  // I2C (INA226 + RTC)
  #define I2C_SDA 41
  #define I2C_SCL 42

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

  auto dumpMem = [](const char* tag){
  Serial.printf("[%s] heap=%u, dma=%u\n",
    tag,
    ESP.getFreeHeap(),
    heap_caps_get_free_size(MALLOC_CAP_DMA));
  };

  dumpMem("boot");
  #if ENABLE_SD
    initSDMMC_1bit();
    dumpMem("after SD");
  #endif
  #if ENABLE_CAMERA
    initCamera();
    dumpMem("after CAM");
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
      config.frame_size   = FRAMESIZE_VGA; // 640*480
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


  // Web handle
  void handleRoot() {
    String html;
    html += "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32-S3 CAM Dashboard</title>";
    html += "<style>";
    html += "body{font-family:Arial,Helvetica,sans-serif;margin:0;background:#0b1020;color:#e8ecff;}";
    html += ".wrap{min-height:100vh;display:flex;justify-content:center;align-items:flex-start;padding:20px;}";
    html += ".container{width:100%;max-width:1100px;}";
    html += ".title{font-size:22px;font-weight:800;text-align:center;margin:6px 0 14px 0;}";
    html += ".card{background:#121a33;border:1px solid rgba(255,255,255,0.10);border-radius:14px;padding:14px;}";
    html += ".row{display:grid;grid-template-columns:1fr;gap:14px;}";
    html += "@media(min-width:900px){.row{grid-template-columns:1.25fr 0.95fr;align-items:start;}}";
    html += ".imgWrap{display:flex;flex-direction:column;gap:10px;align-items:center;}";
    html += "img{width:100%;max-width:760px;border-radius:12px;border:1px solid rgba(255,255,255,0.12);background:#000;}";
    html += ".btns{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin-top:4px;}";
    html += "button{background:#2b5cff;color:#fff;border:0;border-radius:10px;padding:10px 14px;font-size:15px;cursor:pointer;}";
    html += "button.sec{background:#26304d;}";
    html += ".grid{display:grid;grid-template-columns:1fr;gap:12px;}";
    html += "@media(min-width:560px){.grid.two{grid-template-columns:1fr 1fr;}}";
    html += ".label{opacity:0.75;font-size:12px;margin-bottom:6px;}";
    html += ".kv{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid rgba(255,255,255,0.08);}";
    html += ".kv:last-child{border-bottom:0;}";
    html += ".k{opacity:0.85;}";
    html += ".v{font-family:Consolas,Menlo,monospace;}";
    html += ".mono{font-family:Consolas,Menlo,monospace;}";
    html += ".muted{opacity:0.7;font-size:12px;text-align:center;margin-top:6px;}";
    html += "</style>";
    html += "</head><body>";

    html += "<div class='wrap'><div class='container'>";
    html += "<div class='title'>ESP32-S3 CAM Dashboard</div>";

    html += "<div class='row'>";

    // LEFT: Camera
    html += "  <div class='card imgWrap'>";
    html += "    <img id='img' src='/jpg?t=0' alt='camera'/>";
    html += "    <div class='btns'>";
    html += "      <button onclick='capture()'>Capture Photo</button>";
    html += "      <button class='sec' onclick='refreshImage()'>Refresh Image</button>";
    html += "    </div>";
    html += "  </div>";

    // RIGHT: Tables
    html += "  <div class='grid'>";

    // IN/OUT side-by-side on medium screens
    html += "    <div class='grid two'>";

    html += "      <div class='card'>";
    html += "        <div class='label'>INPUT (INA226 #1)</div>";
    html += "        <div class='kv'><div class='k'>Vin (V)</div><div class='v' id='vin'>-</div></div>";
    html += "        <div class='kv'><div class='k'>Iin (A)</div><div class='v' id='iin'>-</div></div>";
    html += "        <div class='kv'><div class='k'>Pin (W)</div><div class='v' id='pin'>-</div></div>";
    html += "      </div>";

    html += "      <div class='card'>";
    html += "        <div class='label'>OUTPUT (INA226 #2)</div>";
    html += "        <div class='kv'><div class='k'>Vout (V)</div><div class='v' id='vout'>-</div></div>";
    html += "        <div class='kv'><div class='k'>Iout (A)</div><div class='v' id='iout'>-</div></div>";
    html += "        <div class='kv'><div class='k'>Pout (W)</div><div class='v' id='pout'>-</div></div>";
    html += "      </div>";

    html += "    </div>";

    // SUMMARY
    html += "    <div class='card'>";
    html += "      <div class='label'>SUMMARY</div>";
    html += "      <div class='kv'><div class='k'>Time</div><div class='v mono' id='ts'>-</div></div>";
    html += "      <div class='kv'><div class='k'>Uptime (min)</div><div class='v' id='upmin'>-</div></div>";
    html += "      <div class='kv'><div class='k'>Efficiency (%)</div><div class='v' id='eff'>-</div></div>";
    html += "      <div class='kv'><div class='k'>Latitude</div><div class='v' id='lat'>-</div></div>";
    html += "      <div class='kv'><div class='k'>Longitude</div><div class='v' id='lng'>-</div></div>";
    html += "      <div class='kv'><div class='k'>Photos saved</div><div class='v' id='photos'>-</div></div>";
    html += "    </div>";

    html += "  </div>"; // end RIGHT grid
    html += "</div>";   // end row

    // JS
    html += "<script>";
    html += "function refreshImage(){document.getElementById('img').src='/jpg?t='+Date.now();}";
    html += "function capture(){fetch('/capture').then(()=>setTimeout(refreshImage,500));}";
    html += "function fmt(x,n){if(x===null||x===undefined||Number.isNaN(Number(x))) return '-'; return Number(x).toFixed(n);}";
    html += "async function refreshStatus(){";
    html += "  try{";
    html += "    const r=await fetch('/json?t='+Date.now());";
    html += "    const d=await r.json();";
    html += "    document.getElementById('ts').textContent = d.ts ?? '-';";
    html += "    document.getElementById('upmin').textContent = (d.uptime_min ?? '-');";
    html += "    document.getElementById('vin').textContent=fmt(d.vin,3);";
    html += "    document.getElementById('iin').textContent=fmt(d.iin,6);";
    html += "    document.getElementById('pin').textContent=fmt(d.pin,6);";
    html += "    document.getElementById('vout').textContent=fmt(d.vout,3);";
    html += "    document.getElementById('iout').textContent=fmt(d.iout,6);";
    html += "    document.getElementById('pout').textContent=fmt(d.pout,6);";
    html += "    document.getElementById('eff').textContent=fmt(d.eff,2);";
    html += "    document.getElementById('lat').textContent=fmt(d.lat,6);";
    html += "    document.getElementById('lng').textContent=fmt(d.lng,6);";
    html += "    document.getElementById('photos').textContent = d.photos ?? '-';";
    html += "  }catch(e){";
    html += "    document.getElementById('ts').textContent='no data';";
    html += "  }";
    html += "}";
    html += "refreshStatus(); setInterval(refreshStatus,2000);";
    html += "</script>";

    html += "</div></div></body></html>";

    server.send(200, "text/html", html);
  }


  // handle JSON
  void handleJSON() {
    MeasurementData snap{};
    if (xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snap = latest;
      xSemaphoreGive(latestMutex);
    }

    unsigned long upMin = millis() / 60000UL;

    String j = "{";
    j += "\"ts\":\"" + String(snap.timestamp) + "\",";
    j += "\"uptime_min\":" + String(upMin) + ",";
    j += "\"vin\":" + String(snap.vin, 3) + ",";
    j += "\"iin\":" + String(snap.iin, 6) + ",";
    j += "\"pin\":" + String(snap.pin, 6) + ",";
    j += "\"vout\":" + String(snap.vout, 3) + ",";
    j += "\"iout\":" + String(snap.iout, 6) + ",";
    j += "\"pout\":" + String(snap.pout, 6) + ",";
    j += "\"eff\":" + String(snap.efficiency, 2) + ",";
    j += "\"lat\":" + String(snap.lat, 6) + ",";
    j += "\"lng\":" + String(snap.lng, 6) + ",";
    j += "\"photos\":" + String(photoCounter);
    j += "}";

    server.send(200, "application/json", j);
  }

  // handleJPG
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

  void i2cScan() {
    Serial.println("I2C scan...");
    uint8_t count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("Found: 0x%02X\n", addr);
        count++;
      }
      delay(2);
    }
    Serial.printf("Done. Found %u device(s)\n", count);
  }

  // SETUP
  void setup() {
    Serial.begin(115200);
    Wire.begin(41, 4);
    Wire.setClock(100000);
    delay(1000);
    Serial.println("\n--- ESP32-S3 CAM + SD_MMC + Web (Modular) ---");
    Serial.printf("psramFound=%d, PSRAM size=%u\n", psramFound(), ESP.getPsramSize());
    i2cScan();

    sdMutex = xSemaphoreCreateMutex();
    camMutex = xSemaphoreCreateMutex();
    latestMutex = xSemaphoreCreateMutex();
    dataQueue = xQueueCreate(10, sizeof(MeasurementData));

  #if ENABLE_RTC
  rtcOK = rtc.begin();
  Serial.printf("rtc.begin() = %d\n", rtcOK);

  if (rtcOK) {
    if (rtc.lostPower()) {
      Serial.println("RTC lost power -> setting time from compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    DateTime now = rtc.now();
    Serial.printf("RTC now: %04d-%02d-%02d %02d:%02d:%02d\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());

    delay(2000);
    DateTime now2 = rtc.now();
    Serial.printf("RTC +2s: %04d-%02d-%02d %02d:%02d:%02d\n",
      now2.year(), now2.month(), now2.day(),
      now2.hour(), now2.minute(), now2.second());
  }
  #endif

  #if ENABLE_GPS
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  #endif

  #if ENABLE_INA226
    inaOK = ina_in.begin() && ina_out.begin();
    if (inaOK) {
      ina_in.setMaxCurrentShunt(0.2, 0.1); // 200mA 0.1ohm ina226
      ina_out.setMaxCurrentShunt(0.2, 0.1); // 200mA 0.1ohm ina226
      Serial.println("INA226 OK");
    } else {
      Serial.println("INA226 not found -> continue without INA");
    }
  #endif

  #if ENABLE_CAMERA
  if (!initCamera()) Serial.println("Camera init failed");
#endif

#if ENABLE_SD
  if (!initSDMMC_1bit()) Serial.println("SD init failed");
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
    server.on("/json", handleJSON);
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
