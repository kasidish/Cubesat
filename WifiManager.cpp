#include "WifiManager.h"

WifiManager::WifiManager() : server(80) {
}

void WifiManager::begin(const char* ssid, const char* password,
                        SensorManager* sensors, CameraManager* cam,
                        SemaphoreHandle_t camMutex,
                        SemaphoreHandle_t latestMutex,
                        MeasurementData* latestData,
                        volatile bool* captureFlag,
                        volatile uint32_t* photoCounter) {
    _sensors = sensors;
    _cam = cam;
    _camMutex = camMutex;
    _latestMutex = latestMutex;
    _latestData = latestData;
    _captureFlag = captureFlag;
    _photoCounter = photoCounter;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Setup routes using lambda or bind
    // Note: We use std::bind to link the callback to this class instance
    //http://192.168.4.1/
    server.on("/", std::bind(&WifiManager::handleRoot, this));
    server.on("/jpg", std::bind(&WifiManager::handleJPG, this));
    server.on("/capture", std::bind(&WifiManager::handleCapture, this));
    server.on("/json", std::bind(&WifiManager::handleJSON, this));
    
    server.begin();
}

void WifiManager::handleClient() {
    server.handleClient();
}

void WifiManager::handleRoot() {
  String html;
  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32-S3 CAM Dashboard</title>";
  html += "<style>";
  html += "body{font-family:Arial,Helvetica,sans-serif;margin:0;background:#0b1020;color:#e8ecff;}";
  html += ".wrap{min-height:100vh;display:flex;justify-content:center;align-items:flex-start;padding:20px;}";
  html += ".container{width:100%;max-width:1100px;}";
  html += ".title{font-size:22px;font-weight:800;text-align:center;margin:6px 0 14px 0;}";
  html += ".card{background:#121a33;border:1px solid rgba(255,255,255,0.10);border-radius:14px;padding:14px;}";
  html += ".row{display:grid;grid-template-columns:1fr;gap:14px;}";
  html += "@media(min-width:900px){.row{grid-template-columns:1.25fr 0.95fr;align-items:start;}}";
  html += ".imgWrap{display:flex;flex-direction:column;gap:10px;align-items:center;}";
  html += "img{width:100%;max-width:760px;border-radius:12px;border:1px solid rgba(255,255,255,0.12);background:#000;}";
  html += ".btns{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin-top:4px;}";
  html += "button{background:#2b5cff;color:#fff;border:0;border-radius:10px;padding:10px 14px;font-size:15px;cursor:pointer;}";
  html += "button.sec{background:#26304d;}";
  html += ".grid{display:grid;grid-template-columns:1fr;gap:12px;}";
  html += "@media(min-width:560px){.grid.two{grid-template-columns:1fr 1fr;}}";
  html += ".label{opacity:0.75;font-size:12px;margin-bottom:6px;}";
  html += ".kv{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid rgba(255,255,255,0.08);}";
  html += ".kv:last-child{border-bottom:0;}";
  html += ".k{opacity:0.85;}";
  html += ".v{font-family:Consolas,Menlo,monospace;}";
  html += ".mono{font-family:Consolas,Menlo,monospace;}";
  html += ".muted{opacity:0.7;font-size:12px;text-align:center;margin-top:6px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='wrap'><div class='container'>";
  html += "<div class='title'>ESP32-S3 CAM Dashboard</div>";
  html += "<div class='row'>";
  // LEFT: Camera
  html += "  <div class='card imgWrap'>";
  html += "    <img id='img' src='/jpg?t=0' alt='camera'/>";
  html += "    <div class='btns'>";
  html += "      <button onclick='capture()'>Capture Photo</button>";
  html += "      <button class='sec' onclick='refreshImage()'>Refresh Image</button>";
  html += "    </div>";
  html += "  </div>";
  // RIGHT: Tables
  html += "  <div class='grid'>";
  // IN/OUT side-by-side on medium screens
  html += "    <div class='grid two'>";
  html += "      <div class='card'>";
  html += "        <div class='label'>INPUT (INA226 #1)</div>";
  html += "        <div class='kv'><div class='k'>Vin (V)</div><div class='v' id='vin'>-</div></div>";
  html += "        <div class='kv'><div class='k'>Iin (A)</div><div class='v' id='iin'>-</div></div>";
  html += "        <div class='kv'><div class='k'>Pin (W)</div><div class='v' id='pin'>-</div></div>";
  html += "      </div>";
  html += "      <div class='card'>";
  html += "        <div class='label'>OUTPUT (INA226 #2)</div>";
  html += "        <div class='kv'><div class='k'>Vout (V)</div><div class='v' id='vout'>-</div></div>";
  html += "        <div class='kv'><div class='k'>Iout (A)</div><div class='v' id='iout'>-</div></div>";
  html += "        <div class='kv'><div class='k'>Pout (W)</div><div class='v' id='pout'>-</div></div>";
  html += "      </div>";
  html += "    </div>";
  // SUMMARY
  html += "    <div class='card'>";
  html += "      <div class='label'>SUMMARY</div>";
  html += "      <div class='kv'><div class='k'>Time</div><div class='v mono' id='ts'>-</div></div>";
  html += "      <div class='kv'><div class='k'>Uptime (min)</div><div class='v' id='upmin'>-</div></div>";
  html += "      <div class='kv'><div class='k'>Efficiency (%)</div><div class='v' id='eff'>-</div></div>";
  html += "      <div class='kv'><div class='k'>Latitude</div><div class='v' id='lat'>-</div></div>";
  html += "      <div class='kv'><div class='k'>Longitude</div><div class='v' id='lng'>-</div></div>";
  html += "      <div class='kv'><div class='k'>Photos saved</div><div class='v' id='photos'>-</div></div>";
  html += "    </div>";
  html += "  </div>"; // end RIGHT grid
  html += "</div>";   // end row
  // JS
  html += "<script>";
  html += "function refreshImage(){document.getElementById('img').src='/jpg?t='+Date.now();}";
  html += "function capture(){fetch('/capture').then(()=>setTimeout(refreshImage,500));}";
  html += "function fmt(x,n){if(x===null||x===undefined||Number.isNaN(Number(x))) return '-'; return Number(x).toFixed(n);}";
  html += "async function refreshStatus(){";
  html += "  try{";
  html += "    const r=await fetch('/json?t='+Date.now());";
  html += "    const d=await r.json();";
  html += "    document.getElementById('ts').textContent = d.ts ?? '-';";
  html += "    document.getElementById('upmin').textContent = (d.uptime_min ?? '-');";
  html += "    document.getElementById('vin').textContent=fmt(d.vin,3);";
  html += "    document.getElementById('iin').textContent=fmt(d.iin,6);";
  html += "    document.getElementById('pin').textContent=fmt(d.pin,6);";
  html += "    document.getElementById('vout').textContent=fmt(d.vout,3);";
  html += "    document.getElementById('iout').textContent=fmt(d.iout,6);";
  html += "    document.getElementById('pout').textContent=fmt(d.pout,6);";
  html += "    document.getElementById('eff').textContent=fmt(d.eff,2);";
  html += "    document.getElementById('lat').textContent=fmt(d.lat,6);";
  html += "    document.getElementById('lng').textContent=fmt(d.lng,6);";
  html += "    document.getElementById('photos').textContent = d.photos ?? '-';";
  html += "  }catch(e){";
  html += "    document.getElementById('ts').textContent='no data';";
  html += "  }";
  html += "}";
  html += "refreshStatus(); setInterval(refreshStatus,2000);";
  html += "</script>";

  html += "</div></div></body></html>";

  server.send(200, "text/html", html);
}

void WifiManager::handleJPG() {
    if (xSemaphoreTake(_camMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        server.send(503, "text/plain", "Camera busy");
        return;
    }

    camera_fb_t* fb = _cam->capture();
    if (!fb) {
        xSemaphoreGive(_camMutex);
        server.send(500, "text/plain", "Camera capture failed");
        return;
    }

    WiFiClient client = server.client();
    server.setContentLength(fb->len);
    server.send(200, "image/jpeg", "");
    client.write(fb->buf, fb->len);

    _cam->returnFrame(fb);
    xSemaphoreGive(_camMutex);
}

void WifiManager::handleCapture() {
    *_captureFlag = true;
    server.send(200, "text/plain", "OK");
}

void WifiManager::handleJSON() {
    MeasurementData snap{};
    if (xSemaphoreTake(_latestMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snap = *_latestData;
        xSemaphoreGive(_latestMutex);
    }

    unsigned long upMin = millis() / 60000UL;

    String j = "{";
    j += "\"ts\":\"" + String(snap.timestamp) + "\",";
    j += "\"uptime_min\":" + String(upMin) + ",";
    j += "\"vin\":" + String(snap.vin, 3) + ",";
    j += "\"iin\":" + String(snap.iin, 6) + ",";
    j += "\"pin\":" + String(snap.pin, 6) + ",";
    j += "\"vout\":" + String(snap.vout, 3) + ",";
    j += "\"iout\":" + String(snap.iout, 6) + ",";
    j += "\"pout\":" + String(snap.pout, 6) + ",";
    j += "\"eff\":" + String(snap.efficiency, 2) + ",";
    j += "\"lat\":" + String(snap.lat, 6) + ",";
    j += "\"lng\":" + String(snap.lng, 6) + ",";
    j += "\"photos\":" + String(*_photoCounter);
    j += "}";

    server.send(200, "application/json", j);
}
