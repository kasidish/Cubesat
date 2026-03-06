#include "MqttService.h"

MqttService::MqttService() : sensors(nullptr), lastPublishTime(0), client(espClient) {
}

void MqttService::begin(SensorService* s) {
    sensors = s;

#if ENABLE_MQTT
    client.setServer(MQTT_BROKER, MQTT_PORT);
    client.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->callback(topic, payload, length);
    });
    Serial.println("MqttService initialized. Waiting for WiFi...");
#endif
}

void MqttService::update() {
#if ENABLE_MQTT
    if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) {
            reconnect();
        }
        client.loop();

        if (millis() - lastPublishTime >= PUBLISH_INTERVAL) {
            if (currentSystemMode == MODE_SENSOR) {
                publishTelemetry();
            }
            lastPublishTime = millis();
        }
    }
#endif
}

void MqttService::reconnect() {
    // Loop until we're reconnected
    if (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP32S3Client-";
        clientId += String(random(0xffff), HEX);
        
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            client.subscribe("cubesat/command");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
        }
    }
}

void MqttService::publishTelemetry() {
    if (!sensors || !client.connected()) return;

    MeasurementData d = sensors->getLatestData();

    String j = "{";
    j += "\"ts\":\"" + String(d.timestamp) + "\",";
    j += "\"vin\":" + String(d.vin, 3) + ",";
    j += "\"iin\":" + String(d.iin, 6) + ",";
    j += "\"pin\":" + String(d.pin, 6) + ",";
    j += "\"vout\":" + String(d.vout, 3) + ",";
    j += "\"iout\":" + String(d.iout, 6) + ",";
    j += "\"pout\":" + String(d.pout, 6) + ",";
    j += "\"eff\":" + String(d.efficiency, 2) + ",";
    j += "\"lat\":" + String(d.lat, 6) + ",";
    j += "\"lng\":" + String(d.lng, 6) + ",";
    j += "\"adc_n0\":" + String(d.adcNormalized[0], 2) + ",";
    j += "\"adc_n1\":" + String(d.adcNormalized[1], 2) + ",";
    j += "\"adc_n2\":" + String(d.adcNormalized[2], 2) + ",";
    j += "\"adc_n3\":" + String(d.adcNormalized[3], 2) + ",";
    j += "\"bat_pct\":" + String(d.estimatedBatteryPct, 1);
    j += "}";

    if (client.publish(MQTT_TOPIC, j.c_str())) {
        Serial.println("Published to " + String(MQTT_TOPIC));
    } else {
        Serial.println("Publish failed");
    }
}

void MqttService::callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    message.trim();
    message.toLowerCase();

    if (String(topic) == "cubesat/command") {
        Serial.println("MQTT Command Received: " + message);
        if (message == "sensor" || message == "mode:sensor" || message == "wakeup") {
            currentSystemMode = MODE_SENSOR;
            Serial.println("Switched to SENSOR MODE");
            client.publish("cubesat/status", "{\"mode\":\"sensor\"}");
        } else if (message == "camera" || message == "mode:camera") {
            currentSystemMode = MODE_CAMERA;
            Serial.println("Switched to CAMERA MODE");
            client.publish("cubesat/status", "{\"mode\":\"camera\"}");
        } else if (message == "sleep" || message == "mode:sleep") {
            currentSystemMode = MODE_SLEEP;
            Serial.println("Switched to SLEEP MODE");
            client.publish("cubesat/status", "{\"mode\":\"sleep\"}");
        }
    }
}
