#include "SensorManager.h"

SensorManager::SensorManager() 
    : ina_in(ADDR_INA_IN), ina_out(ADDR_INA_OUT), gpsSerial(1), rtcOK(false), inaOK(false) {
}

void SensorManager::begin() {
    // Initialize I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // Initialize RTC
    rtcOK = rtc.begin();
    if (!rtcOK) {
        Serial.println("RTC not found");
    }

    // Initialize GPS
    gpsSerial.begin(115200, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Initialize INA226
    inaOK = ina_in.begin() && ina_out.begin();
    if (inaOK) {
        ina_in.setMaxCurrentShunt(0.2, 0.1);
        ina_out.setMaxCurrentShunt(0.2, 0.1);
        Serial.println("INA226 OK");
    } else {
        Serial.println("INA226 not found");
    }
}

void SensorManager::makeTimestamp(char* out, size_t outSize) {
    if (rtcOK) {
        DateTime now = rtc.now();
        snprintf(out, outSize, "%04d-%02d-%02dT%02d:%02d:%02d",
                 now.year(), now.month(), now.day(),
                 now.hour(), now.minute(), now.second());
    } else {
        snprintf(out, outSize, "ms_%lu", (unsigned long)millis());
    }
}

MeasurementData SensorManager::readData() {
    MeasurementData d = {};
    
    // 1. Timestamp
    makeTimestamp(d.timestamp, sizeof(d.timestamp));

    // 2. Power Data (INA226)
    if (inaOK) {
        d.vin = ina_in.getBusVoltage();
        d.iin = ina_in.getCurrent();
        d.pin = ina_in.getPower();
        d.vout = ina_out.getBusVoltage();
        d.iout = ina_out.getCurrent();
        d.pout = ina_out.getPower();
        d.efficiency = (d.pin > 0.000001f) ? (d.pout / d.pin) * 100.0f : 0.0f;
    }

    // 3. GPS Data
    // Feed GPS parser
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }
    d.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    d.lng = gps.location.isValid() ? gps.location.lng() : 0.0;

    return d;
}
