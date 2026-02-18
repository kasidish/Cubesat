#include "WebService.h"

WebService::WebService() : server(80) {}

void WebService::begin(SharedData* s) {
    shared = s;

    server.on("/json", [this]() {
        handleJSON();
    });

    server.begin();
}

void WebService::handle() {
    server.handleClient();
}

void WebService::handleJSON() {
    MeasurementData d = shared->get();

    String j = "{";
    j += "\"vin\":" + String(d.vin,3) + ",";
    j += "\"iin\":" + String(d.iin,3) + ",";
    j += "\"pin\":" + String(d.pin,3) + ",";
    j += "\"eff\":" + String(d.efficiency,2);
    j += "}";

    server.send(200,"application/json",j);
}
