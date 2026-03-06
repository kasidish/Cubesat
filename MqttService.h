#ifndef MQTT_SERVICE_H
#define MQTT_SERVICE_H

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DataModel.h"
#include "SensorService.h"

class MqttService {
public:
    MqttService();
    void begin(SensorService* s);
    void update();

private:
    void reconnect();
    void publishTelemetry();
    void callback(char* topic, byte* payload, unsigned int length);

    WiFiClient espClient;
    PubSubClient client;
    SensorService* sensors;
    unsigned long lastPublishTime;
    const unsigned long PUBLISH_INTERVAL = 5000; // Publish every 5 seconds
};

#endif
