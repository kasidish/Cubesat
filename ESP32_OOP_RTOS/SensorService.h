#pragma once
#include "DataModel.h"

class SensorService {
private:
    static void task(void* param);
    SharedData* shared;

public:
    void begin(SharedData* s);
};