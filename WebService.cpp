#include "WebService.h"
#include "MqttService.h"
#include "esp_wpa2.h"

WebService::WebService() : server(80), sensors(nullptr), camera(nullptr), mqtt(nullptr) {}

void WebService::begin(SensorService* s, CameraService* c, MqttService* m) {
    sensors = s;
    camera = c;
    mqtt = m;

#if ENABLE_WIFI
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Cubesat_GROUP4", "12345678");

#if ENABLE_WIFI_ENTERPRISE
    WiFi.disconnect(true);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_wpa2_ent_enable();
    WiFi.begin(WIFI_SSID); // For eduroam, SSID is often "eduroam"
#else
    WiFi.begin(WIFI_SSID, WIFI_PASS);
#endif
    Serial.print("Connecting to WiFi ");
    
    // Wait up to 10 seconds for STA connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println("");
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! IP Address: ");
        Serial.println(WiFi.localIP());

        // Sync Time using NTP
        configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
        Serial.println("Waiting for NTP time sync...");
        struct tm timeinfo;
        int retries = 0;
        while (!getLocalTime(&timeinfo) && retries < 10) {
            delay(500);
            Serial.print(".");
            retries++;
        }
        
        if (retries < 10) {
            Serial.println("\nTime synchronized.");
            if (sensors) {
                sensors->syncTime(&timeinfo);
            }
        } else {
            Serial.println("\nFailed to obtain NTP time.");
        }

    } else {
        Serial.println("WiFi STA Connection Failed. Running in AP Mode only.");
        Serial.print("AP IP Address: ");
        Serial.println(WiFi.softAPIP());
    }

    server.on("/", HTTP_GET, std::bind(&WebService::handleRoot, this));
    server.on("/json", HTTP_GET, std::bind(&WebService::handleJSON, this));
    server.on("/jpg", HTTP_GET, std::bind(&WebService::handleJPG, this));
    server.on("/capture", HTTP_GET, std::bind(&WebService::handleCapture, this));
    server.on("/status", HTTP_GET, std::bind(&WebService::handleStatus, this)); // Plain text
    server.on("/setMode", HTTP_GET, std::bind(&WebService::handleSetMode, this));

    server.begin();
    Serial.println("WebService started");
#endif
}

void WebService::update() {
    server.handleClient();
}

void WebService::handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32-S3 Cubesat</title>";
    html += "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;} img{max-width:100%;} .status{border:1px solid #444;padding:10px;margin:10px auto;width:300px;text-align:left;}</style>";
    html += "</head><body>";
    html += "<h1>Cubesat Dashboard</h1>";
    
    html += "<div class='status'>";
    html += "<h3>Device Status</h3>";
    html += "WiFi : <span id='st_wifi'>Loading...</span><br/>";
    html += "MQTT : <span id='st_mqtt'>Loading...</span><br/>";
    html += "Node-RED : <span id='st_nr'>Loading...</span><br/>";
    html += "GPS Sats : <span id='st_gps'>Loading...</span><br/>";
    html += "GPS SNR : <span id='st_snr'>Loading...</span><br/>";
    html += "Battery : <span id='st_bat'>Loading...</span><br/>";
    html += "Mode : <span id='mode_str'>Loading...</span><br/>";
    html += "Last Photo : <span id='st_photo'>-</span><br/>";
    html += "</div>";

    html += "<div style='margin-bottom: 20px;'>";
    html += "<button onclick=\"fetch('/setMode?m=sensor')\">Sensor Mode</button> ";
    html += "<button onclick=\"fetch('/setMode?m=camera')\">Camera Mode</button> ";
    html += "<button onclick=\"fetch('/setMode?m=sleep')\">Sleep</button>";
    html += "</div>";
    
    html += "<img id='cam' src='/jpg' /><br/><br/>";
    html += "<button onclick=\"fetch('/capture')\">Capture to SD</button> ";
    html += "<button onclick=\"document.getElementById('cam').src='/jpg?t='+Date.now()\">Refresh</button>";
    
    html += "<div id='data'>Loading...</div>";
    html += "<script>setInterval(async()=>{";
    html += "  try {";
    html += "    const r=await fetch('/json');const d=await r.json();";
    html += "    document.getElementById('data').innerText = JSON.stringify(d,null,2);";
    html += "    document.getElementById('mode_str').innerText = d.mode_str;";
    html += "    if(d.ts){";
    html += "      var parts=d.ts.split('T');";
    html += "      var timeClean=parts[1]?parts[1].replace(/:/g,'-'):'';";
    html += "      document.getElementById('st_photo').innerText='img_'+parts[0]+'_Time_'+timeClean;";
    html += "    }";
    html += "    document.getElementById('st_wifi').innerText = d.wifi_connected ? 'Connected' : 'Disconnected';";
    html += "    document.getElementById('st_mqtt').innerText = d.mqtt_connected ? 'Connected' : 'Disconnected';";
    html += "    document.getElementById('st_nr').innerText = d.mqtt_connected ? 'Connected (via MQTT)' : 'Disconnected';";
    html += "    document.getElementById('st_gps').innerText = d.satellites;";
    html += "    document.getElementById('st_snr').innerText = d.snr || '-';";
    html += "    var batStr = d.vin ? (d.vin.toFixed(2) + 'V') : 'N/A';";
    html += "    if(d.batt_soc !== undefined) batStr += ' | ' + d.batt_soc.toFixed(1) + '%';";
    html += "    document.getElementById('st_bat').innerText = batStr;";
    html += "  } catch(e) {}";
    html += "},2000);</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void WebService::handleJSON() {
    if (!sensors) { server.send(500); return; }
    MeasurementData d = sensors->getLatestData();

    String j = "{";
    j += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    bool mqttConn = mqtt ? mqtt->isConnected() : false;
    j += "\"mqtt_connected\":" + String(mqttConn ? "true" : "false") + ",";
    j += "\"mode\":" + String(currentSystemMode) + ",";
    j += "\"mode_str\":\"" + String(currentSystemMode == MODE_SENSOR ? "Sensor" : currentSystemMode == MODE_CAMERA ? "Camera" : "Sleep") + "\",";
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
    j += "\"satellites\":" + String(d.satellites) + ",";
    j += "\"snr\":\"" + String(d.snrData) + "\",";
    j += "\"batt_soc\":" + String(d.battSoC, 2) + ",";
    j += "\"adc0\":" + String(d.adcValues[0]) + ",";
    j += "\"adc1\":" + String(d.adcValues[1]) + ",";
    j += "\"adc2\":" + String(d.adcValues[2]) + ",";
    j += "\"adc3\":" + String(d.adcValues[3]);
    j += "}";
    server.send(200, "application/json", j);
}

void WebService::handleJPG() {
    if (!camera) { server.send(503, "text/plain", "No Camera"); return; }
    
    camera_fb_t* fb = camera->getFrame();
    if (!fb) {
        server.send(503, "text/plain", "Camera Busy/Fail");
        return;
    }

    WiFiClient client = server.client();
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");
    client.write(fb->buf, fb->len);
    
    camera->returnFrame(fb);
}

void WebService::handleCapture() {
    if (camera) {
        camera->triggerCapture();
        server.send(200, "text/plain", "Capture Requested");
    } else {
        server.send(500, "text/plain", "No Camera");
    }
}

void WebService::handleStatus() {
    handleJSON(); // Reuse JSON for status
}

void WebService::handleSetMode() {
    if (server.hasArg("m")) {
        String md = server.arg("m");
        md.toLowerCase();
        if (md == "sensor") {
            currentSystemMode = MODE_SENSOR;
            Serial.println("Web: Switched to SENSOR MODE");
            server.send(200, "text/plain", "Mode set to Sensor");
            return;
        } else if (md == "camera") {
            currentSystemMode = MODE_CAMERA;
            Serial.println("Web: Switched to CAMERA MODE");
            server.send(200, "text/plain", "Mode set to Camera");
            return;
        } else if (md == "sleep" || md == "wakeup") {
            currentSystemMode = (md == "sleep") ? MODE_SLEEP : MODE_SENSOR;
            Serial.println("Web: Switched to " + String(md));
            server.send(200, "text/plain", "Mode set to " + String(md));
            return;
        }
    }
    server.send(400, "text/plain", "Invalid mode parameter");
}
