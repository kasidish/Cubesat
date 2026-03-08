#include "SensorService.h"

SensorService::SensorService() 
    : ina_in(0x41), ina_out(0x44), gpsSerial(1), inaInOK(false), inaOutOK(false), rtcOK(false), 
      dataQueuePtr(nullptr), bootTimeMs(0), totalSatsInView(0), 
      socAccum(0.0f), lastSocMs(0), socInitialized(false) {
    mutex = xSemaphoreCreateMutex();
    nmeaSentence = "";
    memset(&latest, 0, sizeof(MeasurementData));
    for (int i = 0; i < 4; i++) {
        adcFiltered[i] = -1.0f;
    }
}

void SensorService::begin() {
    Wire.begin(I2C_SDA, I2C_SCL); // 41, 42
    Wire.setClock(100000);
    Wire.setTimeOut(100);
    
#if ENABLE_RTC
    rtcOK = rtc.begin();
    if (rtcOK && rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
#endif

#if ENABLE_INA226
    Serial.println("Initializing INA226 IN (0x41)...");
    inaInOK = ina_in.begin();
    if (inaInOK) {
        ina_in.setMaxCurrentShunt(0.2, 0.1);
        Serial.println("INA226 IN: OK");
    } else {
        Serial.println("INA226 IN: FAILED/NOT FOUND");
    }
    
    Serial.println("Initializing INA226 OUT (0x44)...");
    inaOutOK = ina_out.begin();
    if (inaOutOK) {
        ina_out.setMaxCurrentShunt(0.2, 0.1);
        Serial.println("INA226 OUT: OK");
    } else {
        Serial.println("INA226 OUT: FAILED/NOT FOUND");
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
    MeasurementData d = {};
    makeTimestamp(d.timestamp, sizeof(d.timestamp));

    if (currentSystemMode == MODE_SENSOR) {
#if ENABLE_INA226
        if (inaInOK) {
            d.vin = ina_in.getBusVoltage();
            d.iin = ina_in.getCurrent();
            d.pin = ina_in.getPower();
        }
        if (inaOutOK) {
            d.vout = ina_out.getBusVoltage();
            d.iout = ina_out.getCurrent();
            d.pout = ina_out.getPower();
        }
        if (inaInOK && inaOutOK) {
            d.efficiency = (d.pin > 0.000001f) ? (d.pout / d.pin) * 100.0f : 0.0f;
        } else {
            d.efficiency = 0.0f;
        }
        
        // Update State of Charge
        updateSoC(d);
#endif

#if ENABLE_GPS
        while (gpsSerial.available()) {
            char c = gpsSerial.read();
            gps.encode(c);
        }
        
        if (gps.location.isValid()) {
            d.lat = gps.location.lat();
            d.lng = gps.location.lng();
        }
        if (gps.satellites.isValid()) {
            d.satellites = gps.satellites.value();
        } else {
            d.satellites = 0;
        }
#endif

        // ADC Reading and Filtering
        float alpha = (millis() - bootTimeMs < 300000) ? 0.05f : 0.2f; 

        for (int i = 0; i < 4; i++) {
            int raw = analogRead(ADC_PINS[i]);
            if (adcFiltered[i] < 0.0f) {
                adcFiltered[i] = raw; 
            } else {
                adcFiltered[i] = (alpha * raw) + ((1.0f - alpha) * adcFiltered[i]);
            }
            
            d.adcValues[i] = (int)adcFiltered[i];
        }

        // Process Comparator Logic
        int activeCount = 0;
        for (int i = 0; i < 4; i++) {
            d.logicLevels[i] = 1.0f - (adcFiltered[i] / 4095.0f);
            if (d.logicLevels[i] > 0.5f) {
                activeCount++;
            }
        }
        d.adcSoC = activeCount * 25.0f;
    }

    // Update local protected copy
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        latest = d;
        xSemaphoreGive(mutex);
    }

    // Send to Queue for Telemetry (Always send heartbeat for timestamp continuity)
    if (dataQueuePtr && *dataQueuePtr) {
        xQueueSend(*dataQueuePtr, &d, pdMS_TO_TICKS(10));
    }
}

void SensorService::updateSoC(MeasurementData& d) {
    unsigned long now = millis();
    if (lastSocMs == 0) {
        lastSocMs = now;
        // First run: Init from voltage
        d.battSoC = getSoCFromVoltage(d.vin);
        socAccum = d.battSoC;
        socInitialized = true;
        Serial.printf("SoC Initialized from Voltage: %.2f%%\n", socAccum);
        return;
    }

    float deltaSeconds = (now - lastSocMs) / 1000.0f;
    lastSocMs = now;

    // 1. Coulomb Counting
    // getCurrent() returns Amps. Positive = Charging, Negative = Discharging (presumably)
    // mAh = A * (sec/3600) * 1000
    float deltaMAh = d.iin * (deltaSeconds / 3600.0f) * 1000.0f;
    socAccum += (deltaMAh / BATT_CAPACITY_MAH) * 100.0f;

    // 2. Voltage Correction (OCV)
    // Only correct if current is very low (idle) to avoid voltage drop error
    if (abs(d.iin) < OCV_IDLE_THRESHOLD) {
        float ocvSoC = getSoCFromVoltage(d.vin);
        // Slowly drift toward OCV to correct Coulomb Counting errors
        socAccum = (socAccum * 0.98f) + (ocvSoC * 0.02f);
    }

    // Clamp
    if (socAccum > 100.0f) socAccum = 100.0f;
    if (socAccum < 0.0f) socAccum = 0.0f;

    d.battSoC = socAccum;
}

float SensorService::getSoCFromVoltage(float v) {
    // Li-ion 2S (8.4V Full, 6.0V Cutoff)
    // Simple lookup table / linear interpolation
    if (v >= 8.4f) return 100.0f;
    if (v <= 6.0f) return 0.0f;

    // Approximated discharge curve for Li-ion 2S
    if (v > 8.1f) return 90.0f + (v - 8.1f) * (10.0f / 0.3f);
    if (v > 7.9f) return 80.0f + (v - 7.9f) * (10.0f / 0.2f);
    if (v > 7.7f) return 70.0f + (v - 7.7f) * (10.0f / 0.2f);
    if (v > 7.5f) return 60.0f + (v - 7.5f) * (10.0f / 0.2f);
    if (v > 7.3f) return 50.0f + (v - 7.3f) * (10.0f / 0.2f);
    if (v > 7.1f) return 40.0f + (v - 7.1f) * (10.0f / 0.2f);
    if (v > 6.9f) return 30.0f + (v - 6.9f) * (10.0f / 0.2f);
    if (v > 6.7f) return 20.0f + (v - 6.7f) * (10.0f / 0.2f);
    if (v > 6.4f) return 10.0f + (v - 6.4f) * (10.0f / 0.3f);
    
    return (v - 6.0f) * (10.0f / 0.4f);
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

void SensorService::syncTime(struct tm* timeinfo) {
#if ENABLE_RTC
    if (rtcOK) {
        rtc.adjust(DateTime(
            timeinfo->tm_year + 1900,
            timeinfo->tm_mon + 1,
            timeinfo->tm_mday,
            timeinfo->tm_hour,
            timeinfo->tm_min,
            timeinfo->tm_sec
        ));
        Serial.println("RTC Synced with NTP Time!");
    } else {
        Serial.println("RTC not OK, cannot sync.");
    }
#else
    Serial.println("RTC not enabled, cannot sync.");
#endif
}
