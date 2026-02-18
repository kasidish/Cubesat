#include "SensorService.h"

SensorService::SensorService() 
    : ina_in(0x40), ina_out(0x41), gpsSerial(1), inaOK(false), rtcOK(false), dataQueuePtr(nullptr) {
    mutex = xSemaphoreCreateMutex();
    memset(&latest, 0, sizeof(MeasurementData));
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

    pinMode(SENSOR_ADC_PIN, INPUT);

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

    // ADC Reading (0-3.3V approx, depending on attenuation)
    // ESP32 ADC is 12-bit (0-4095)
    d.adcValue = analogRead(SENSOR_ADC_PIN);
    d.adcVoltage = (d.adcValue / 4095.0f) * 3.3f; // Basic calibration

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
