#include <Arduino.h>
#include "SensorService.h"

void SensorService::begin(SharedData* s) {
    shared = s;
    xTaskCreatePinnedToCore(
        task,
        "SensorTask",
        4096,
        this,
        2,
        NULL,
        0
    );
}

void SensorService::task(void* param) {
    SensorService* self = (SensorService*)param;

    for (;;) {
        MeasurementData d{};

        snprintf(d.timestamp, sizeof(d.timestamp),
                 "ms_%lu", millis());

        // 🔥 ตรงนี้ใส่ INA226 / GPS ของจริงได้เลย
        d.vin = random(100, 200) / 10.0;
        d.iin = random(0, 100) / 100.0;
        d.pin = d.vin * d.iin;
        d.efficiency = random(80, 95);

        self->shared->update(d);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}