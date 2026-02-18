#pragma once
#include <WebServer.h>
#include "DataModel.h"

class WebService {
private:
    WebServer server;
    SharedData* shared;

    void handleJSON();

public:
    WebService();
    void begin(SharedData* s);
    void handle();
};