#include <Arduino.h>
#include "DataModel.h"
#include "SensorService.h"
#include "CameraService.h"
#include "TelemetryService.h"
#include "WebService.h"
#include "MqttService.h"

// Global Services
SensorService sensorService;
CameraService cameraService;
TelemetryService telemetryService;
WebService webService;
MqttService mqttService;

// System Mode State
OperationMode currentSystemMode = MODE_SENSOR; // Default to sensor mode

// Shared Queue
QueueHandle_t dataQueue;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("ESP32-S3 OOP Cubesat");

    // Initialize Shared Queue
    dataQueue = xQueueCreate(10, sizeof(MeasurementData));
    if (dataQueue == NULL) {
        Serial.println("Error creating data queue!");
    }
    
    // Inject Queue into SensorService
    sensorService.setDataQueue(&dataQueue);

    // Initialize Services
    // Order matters mostly for dependencies (telemetry needs SD, camera needs PSRAM check inside)
    
    // 1. Camera
    if (!cameraService.begin()) {
        Serial.println("Camera Init Failed");
    } else {
        Serial.println("Camera Init OK");
    }

    // 2. Sensors (I2C, GPS)
    sensorService.begin();
    
    // 3. Telemetry (SD Card) - Depends on Queue and Camera (for photos)
    telemetryService.begin(&dataQueue, &cameraService);

    // 4. Web Service - Depends on Sensors (for data) and Camera (for streaming)
    webService.begin(&sensorService, &cameraService);

    // 5. MQTT Service
    mqttService.begin(&sensorService);
}

void loop() {
    // Web Server is not purely async in this simple implementation, 
    // it needs a handleClient loop.
    webService.update();
    mqttService.update();
    vTaskDelay(pdMS_TO_TICKS(10));
}
