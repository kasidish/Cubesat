#include "SensorService.h"

SensorService::SensorService() 
    : ina_in(0x40), ina_out(0x41), gpsSerial(1), inaOK(false), rtcOK(false), dataQueuePtr(nullptr), bootTimeMs(0) {
    mutex = xSemaphoreCreateMutex();
    memset(&latest, 0, sizeof(MeasurementData));
    for (int i = 0; i < 4; i++) {
        adcFiltered[i] = -1.0f;
    }
}

void SensorService::begin() {
    Wire.begin(I2C_SDA, I2C_SCL); // 41, 42
    Wire.setClock(100000);

#if ENABLE_RTC
    rtcOK = rtc.begin();
    if (rtcOK && rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
#endif

#if ENABLE_INA226
    inaOK = ina_in.begin() && ina_out.begin();
    if (inaOK) {
        ina_in.setMaxCurrentShunt(0.2, 0.1);
        ina_out.setMaxCurrentShunt(0.2, 0.1);
    }
#endif

#if ENABLE_GPS
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

    for (int i = 0; i < 4; i++) {
        pinMode(ADC_PINS[i], INPUT);
    }
    bootTimeMs = millis();

    xTaskCreatePinnedToCore(SensorService::task, "SensorTask", 4096, this, 2, NULL, 0);
}

MeasurementData SensorService::getLatestData() {
    MeasurementData d;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        d = latest;
        xSemaphoreGive(mutex);
    } else {
        memset(&d, 0, sizeof(d)); // Timeout
    }
    return d;
}

void SensorService::task(void* param) {
    SensorService* self = (SensorService*)param;
    for (;;) {
        self->loop();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Sample rate
    }
}

void SensorService::loop() {
    if (currentSystemMode != MODE_SENSOR) {
        return; // Skip hardware reading and data generation
    }

    MeasurementData d = {};
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
    if (gps.location.isValid()) {
        d.lat = gps.location.lat();
        d.lng = gps.location.lng();
    }
#endif

    // ADC Reading and Filtering
    float alpha = (millis() - bootTimeMs < 300000) ? 0.05f : 0.2f; // 5 min slow EMA, then fast EMA

    for (int i = 0; i < 4; i++) {
        int raw = analogRead(ADC_PINS[i]);
        if (adcFiltered[i] < 0.0f) {
            adcFiltered[i] = raw; // Init on first read
        } else {
            adcFiltered[i] = (alpha * raw) + ((1.0f - alpha) * adcFiltered[i]);
        }
        
        d.adcValues[i] = (int)adcFiltered[i];
        d.adcVoltages[i] = (d.adcValues[i] / 4095.0f) * 3.3f; // ESP32 is 12-bit (0-4095)
        
        // Normalize 0V -> 1, 3.3V -> 0
        d.adcNormalized[i] = 1.0f - (d.adcVoltages[i] / 3.3f);
        if (d.adcNormalized[i] < 0.0f) d.adcNormalized[i] = 0.0f;
        if (d.adcNormalized[i] > 1.0f) d.adcNormalized[i] = 1.0f;
    }

    // Battery Estimation using non-linear 2S curve
    if (d.adcNormalized[0] > 0.05f) { 
        d.estimatedBatteryPct = 70.0f + (d.adcNormalized[0] * 30.0f); 
    } else if (d.adcNormalized[1] > 0.05f) { 
        d.estimatedBatteryPct = 50.0f + (d.adcNormalized[1] * 20.0f);
    } else if (d.adcNormalized[2] > 0.05f) { 
        d.estimatedBatteryPct = 30.0f + (d.adcNormalized[2] * 20.0f);
    } else if (d.adcNormalized[3] > 0.05f) { 
        d.estimatedBatteryPct = 10.0f + (d.adcNormalized[3] * 20.0f);
    } else { 
        d.estimatedBatteryPct = d.adcNormalized[3] * 10.0f;
    }

    // Update local protected copy
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        latest = d;
        xSemaphoreGive(mutex);
    }

    // Send to Queue for Telemetry
    if (dataQueuePtr && *dataQueuePtr) {
        xQueueSend(*dataQueuePtr, &d, pdMS_TO_TICKS(10));
    }
}

void SensorService::makeTimestamp(char* out, size_t outSize) {
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
