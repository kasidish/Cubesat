#pragma once
#include <Arduino.h>

struct MeasurementData {
    char timestamp[32];
    float vin = 0;
    float iin = 0;
    float pin = 0;
    float vout = 0;
    float iout = 0;
    float pout = 0;
    float efficiency = 0;
    double lat = 0;
    double lng = 0;
};

class SharedData {
private:
    MeasurementData data;
    SemaphoreHandle_t mutex;

public:
    void begin() {
        mutex = xSemaphoreCreateMutex();
    }

    void update(const MeasurementData& d) {
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            data = d;
            xSemaphoreGive(mutex);
        }
    }

    MeasurementData get() {
        MeasurementData copy{};
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            copy = data;
            xSemaphoreGive(mutex);
        }
        return copy;
    }
};