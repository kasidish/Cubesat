cubesat.cpp is FreeRTOS by use ESP-IDF C++ just only for INA226 Sensor test

esp32cubesat.c is Arduino IDE (FreeRTOS) by use ESP32-S3 for All system design (OV2640, INA226, GPS, SD Card, RTC, WiFi, Web Server) with OOP Object Oriented Programming ;
by using c (embeded c) coding

so it separate all system each file so it easy to debug and test and modify :

* .h for header, class, declare variable, declare function
* .cpp for function, process
* .c for main program, import class and function from .h and .cpp

- SensorManager.h/.cpp : INA226 Sensor + GPS + RTC
- CameraManager.h/.cpp : OV2640 Camera
- WifiManager.h/.cpp : WiFi + Web Server
- esp32cubesat.c (main)

if u want to test ESP32-S3 for All system design (OV2640, INA226, GPS, SD Card, RTC, WiFi, Web Server) ; by using c (embeded c) coding
use esp32cubesat.c (main)

if u want to test INA226 Sensor test by using ESP-IDF C++ coding
use cubesat.cpp

then if it have some error or bug, fix it and test again