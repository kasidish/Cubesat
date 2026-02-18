#pragma once
#include "DataModel.h"

class TelemetryService {
private:
    static void task(void* param);
    SharedData* shared;

public:
    void begin(SharedData* s);
};