#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <Arduino.h>
#include "esp_camera.h"

class CameraManager {
public:
    CameraManager();
    bool begin();
    camera_fb_t* capture();
    void returnFrame(camera_fb_t* fb);

private:
    // Pin Definitions
    static const int PWDN_GPIO_NUM = -1;
    static const int RESET_GPIO_NUM = -1;
    static const int XCLK_GPIO_NUM = 15;
    static const int SIOD_GPIO_NUM = 4;
    static const int SIOC_GPIO_NUM = 5;

    static const int Y9_GPIO_NUM = 16;
    static const int Y8_GPIO_NUM = 17;
    static const int Y7_GPIO_NUM = 18;
    static const int Y6_GPIO_NUM = 12;
    static const int Y5_GPIO_NUM = 10;
    static const int Y4_GPIO_NUM = 8;
    static const int Y3_GPIO_NUM = 9;
    static const int Y2_GPIO_NUM = 11;
    static const int VSYNC_GPIO_NUM = 6;
    static const int HREF_GPIO_NUM = 7;
    static const int PCLK_GPIO_NUM = 13;
};

#endif
