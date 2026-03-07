#include "TelemetryService.h"

TelemetryService::TelemetryService() : dataQueuePtr(nullptr), camera(nullptr), photoCounter(0) {
    sdMutex = xSemaphoreCreateMutex();
}

bool TelemetryService::begin(QueueHandle_t* q, CameraService* cam) {
    dataQueuePtr = q;
    camera = cam;

#if ENABLE_SD
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("SD_MMC mount failed");
    } else {
        Serial.println("SD_MMC mounted");
        if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
        
        // CSV Header
        if (!SD_MMC.exists("/datalog.csv")) {
             File f = SD_MMC.open("/datalog.csv", FILE_WRITE);
             if (f) {
                 f.println("Timestamp,Mode,Vin(V),Iin(A),Pin(W),Vout(V),Iout(A),Pout(W),Efficiency(%),Latitude,Longitude,Satellites,SNR,ADC0,ADC1,ADC2,ADC3,SoC(%)");
                 f.close();
             }
        }
    }
#endif

    xTaskCreatePinnedToCore(TelemetryService::task, "TelemetryTask", 4096, this, 1, NULL, 0);
    return true;
}

void TelemetryService::task(void* param) {
    TelemetryService* self = (TelemetryService*)param;
    for (;;) {
        self->loop();
    }
}

void TelemetryService::loop() {
    MeasurementData d;
    // Wait for data from SensorService
    if (dataQueuePtr && *dataQueuePtr && xQueueReceive(*dataQueuePtr, &d, pdMS_TO_TICKS(100)) == pdPASS) {
        latestTimestamp = String(d.timestamp); // Save for photo filename
        logToSerial(d); // Serial Studio
        logToSD(d);     // SD Card
    }

    // Check for photo request
    savePhotoIfRequested();
}

void TelemetryService::logToSerial(const MeasurementData& d) {
    const char* modeStr = (currentSystemMode == MODE_SENSOR) ? "SENSOR" :
                          (currentSystemMode == MODE_CAMERA) ? "CAMERA" : "SLEEP";
    Serial.printf("/*%s,%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f,%d,%s,%d,%d,%d,%d,%.2f*/\n",
        d.timestamp, modeStr,
        d.vin, d.iin, d.pin,
        d.vout, d.iout, d.pout,
        d.efficiency,
        d.lat, d.lng,
        d.satellites, d.snrData,
        d.adcValues[0], d.adcValues[1], d.adcValues[2], d.adcValues[3], d.battSoC
    );
}

void TelemetryService::logToSD(const MeasurementData& d) {
#if ENABLE_SD
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        File f = SD_MMC.open("/datalog.csv", FILE_APPEND);
        if (f) {
            const char* modeStr = (currentSystemMode == MODE_SENSOR) ? "SENSOR" :
                                  (currentSystemMode == MODE_CAMERA) ? "CAMERA" : "SLEEP";
            f.printf("%s,%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f,%d,%s,%d,%d,%d,%d,%.2f\n",
                d.timestamp, modeStr,
                d.vin, d.iin, d.pin,
                d.vout, d.iout, d.pout,
                d.efficiency,
                d.lat, d.lng,
                d.satellites, d.snrData,
                d.adcValues[0], d.adcValues[1], d.adcValues[2], d.adcValues[3], d.battSoC
            );
            f.close();
        }
        xSemaphoreGive(sdMutex);
    }
#endif
}

void TelemetryService::savePhotoIfRequested() {
#if ENABLE_SD && ENABLE_CAMERA
    if (camera && camera->isCaptureRequested()) {
        camera->clearCaptureRequest();

        // Get frame
        camera_fb_t* fb = camera->getFrame();
        if (!fb) return;

        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
            char filename[64];
            // Format: img_2026-03-07_Time_22-30-01_0.jpg (FAT32 safe, human-readable)
            char ts[32];
            strncpy(ts, latestTimestamp.c_str(), sizeof(ts) - 1);
            ts[sizeof(ts) - 1] = '\0';
            for (int i = 0; ts[i]; i++) {
                if (ts[i] == ':') ts[i] = '-'; // FAT32: no colons allowed
                if (ts[i] == 'T') ts[i] = '\0'; // Split at T
            }
            // ts now holds date part only (e.g. "2026-03-07")
            // Find time part after the original T position
            const char* timePart = latestTimestamp.c_str();
            while (*timePart && *timePart != 'T') timePart++;
            if (*timePart == 'T') timePart++; // skip T
            // Sanitize time part into a separate buffer
            char timeTs[16] = "";
            strncpy(timeTs, timePart, sizeof(timeTs) - 1);
            for (int i = 0; timeTs[i]; i++) if (timeTs[i] == ':') timeTs[i] = '-';
            snprintf(filename, sizeof(filename), "/photos/img_%s_Time_%s_%lu.jpg", ts, timeTs, (unsigned long)(photoCounter));
            
            File file = SD_MMC.open(filename, FILE_WRITE);
            if (file) {
                file.write(fb->buf, fb->len);
                file.close();
                photoCounter++;
                Serial.printf("Saved %s\n", filename);
            }
            xSemaphoreGive(sdMutex);
        }
        camera->returnFrame(fb);
    }
#endif
}
