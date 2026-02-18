#include <Arduino.h>
#include "TelemetryService.h"

void TelemetryService::begin(SharedData* s) {
    shared = s;
    xTaskCreatePinnedToCore(
        task,
        "TelemetryTask",
        4096,
        this,
        1,
        NULL,
        0
    );
}

void TelemetryService::task(void* param) {
    TelemetryService* self = (TelemetryService*)param;

    for (;;) {
        MeasurementData d = self->shared->get();

        Serial.printf(
            "{\"vin\":%.3f,"
            "\"iin\":%.3f,"
            "\"pin\":%.3f,"
            "\"eff\":%.2f}\n",
            d.vin,
            d.iin,
            d.pin,
            d.efficiency
        );

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}