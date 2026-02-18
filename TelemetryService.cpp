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
                 f.println("Timestamp,Vin(V),Iin(A),Pin(W),Vout(V),Iout(A),Pout(W),Efficiency(%),Latitude,Longitude,ADC_Raw,ADC_Volts");
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
        logToSerial(d); // Serial Studio
        logToSD(d);     // SD Card
    }

    // Check for photo request
    savePhotoIfRequested();
}

void TelemetryService::logToSerial(const MeasurementData& d) {
    // Print in CSV format for Serial Studio
    // Serial Studio format: /\*.*\*\// // (Comments)
    // Or just plain CSV lines if configured: %s,%.2f,...
    // Here we match the SD Card format which is usually compatible if headers match
    Serial.printf("/*%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f,%d,%.2f*/\n",
        d.timestamp, 
        d.vin, d.iin, d.pin, 
        d.vout, d.iout, d.pout, 
        d.efficiency, 
        d.lat, d.lng,
        d.adcValue, d.adcVoltage
    );
}

void TelemetryService::logToSD(const MeasurementData& d) {
#if ENABLE_SD
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        File f = SD_MMC.open("/datalog.csv", FILE_APPEND);
        if (f) {
            f.printf("%s,%.3f,%.6f,%.6f,%.3f,%.6f,%.6f,%.2f,%.6f,%.6f,%d,%.2f\n",
                d.timestamp, 
                d.vin, d.iin, d.pin, 
                d.vout, d.iout, d.pout, 
                d.efficiency, 
                d.lat, d.lng,
                d.adcValue, d.adcVoltage
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
            snprintf(filename, sizeof(filename), "/photos/img_%lu.jpg", millis());
            
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
