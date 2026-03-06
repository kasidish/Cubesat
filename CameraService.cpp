#include "CameraService.h"
#include "esp_heap_caps.h"

CameraService::CameraService() : captureRequested(false) {
    mutex = xSemaphoreCreateMutex();
}

bool CameraService::begin() {
#if !ENABLE_CAMERA
    return true;
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = CAM_Y2; config.pin_d1 = CAM_Y3; config.pin_d2 = CAM_Y4; config.pin_d3 = CAM_Y5;
    config.pin_d4 = CAM_Y6; config.pin_d5 = CAM_Y7; config.pin_d6 = CAM_Y8; config.pin_d7 = CAM_Y9;
    config.pin_xclk = CAM_XCLK; config.pin_pclk = CAM_PCLK;
    config.pin_vsync = CAM_VSYNC; config.pin_href = CAM_HREF;
    config.pin_sscb_sda = CAM_SIOD; config.pin_sscb_scl = CAM_SIOC;
    config.pin_pwdn = CAM_PWDN; config.pin_reset = CAM_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 14;
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_LATEST;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }
    return true;
}

void CameraService::triggerCapture() {
    if (currentSystemMode == MODE_CAMERA) {
        captureRequested = true;
    }
}

bool CameraService::isCaptureRequested() {
    return captureRequested;
}

void CameraService::clearCaptureRequest() {
    captureRequested = false;
}

camera_fb_t* CameraService::getFrame() {
#if !ENABLE_CAMERA
    return nullptr;
#endif
    if (currentSystemMode != MODE_CAMERA) {
        return nullptr;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return nullptr;
    }
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(mutex);
    return fb;
}

void CameraService::returnFrame(camera_fb_t* fb) {
    if (fb) esp_camera_fb_return(fb);
}
