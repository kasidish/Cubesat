#include "WebService.h"

WebService::WebService() : server(80), sensors(nullptr), camera(nullptr) {}

void WebService::begin(SensorService* s, CameraService* c) {
    sensors = s;
    camera = c;

#if ENABLE_WIFI
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32S3-CAM", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, std::bind(&WebService::handleRoot, this));
    server.on("/json", HTTP_GET, std::bind(&WebService::handleJSON, this));
    server.on("/jpg", HTTP_GET, std::bind(&WebService::handleJPG, this));
    server.on("/capture", HTTP_GET, std::bind(&WebService::handleCapture, this));
    server.on("/status", HTTP_GET, std::bind(&WebService::handleStatus, this)); // Plain text

    server.begin();
    Serial.println("WebService started");
#endif
}

void WebService::update() {
    server.handleClient();
}

void WebService::handleRoot() {
    // Keep it simple or load from SPIFFS ideally. For now, embedding as per original.
    // Truncated for brevity, reusing the logic from original but calling APIs.
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>ESP32-S3 Cubesat</title>";
    html += "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;} img{max-width:100%;}</style>";
    html += "</head><body>";
    html += "<h1>Cubesat Dashboard</h1>";
    html += "<img id='cam' src='/jpg' /><br/><br/>";
    html += "<button onclick=\"fetch('/capture')\">Capture to SD</button> ";
    html += "<button onclick=\"document.getElementById('cam').src='/jpg?t='+Date.now()\">Refresh</button>";
    html += "<div id='data'>Loading...</div>";
    html += "<script>setInterval(async()=>{";
    html += "  const r=await fetch('/json');const d=await r.json();";
    html += "  document.getElementById('data').innerText = JSON.stringify(d,null,2);";
    html += "},2000);</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void WebService::handleJSON() {
    if (!sensors) { server.send(500); return; }
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
    j += "\"adc_val\":" + String(d.adcValue) + ",";
    j += "\"adc_v\":" + String(d.adcVoltage, 2);
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
