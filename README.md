# ESP32-S3 OOP Cubesat Project

## Architecture Overview

The project uses Object-Oriented Design (OOD) where each hardware/functional block is a **Service** class. Services communicate via a **FreeRTOS Queue** and a **Shared Mutex**, running concurrently on dual cores.

### File Structure
| File | Responsibility |
| --- | --- |
| `CubesatProject.ino` | Entry point — initializes all services and runs the main loop |
| `DataModel.h` | Shared config (`ENABLE_*` switches), WiFi/MQTT credentials, `MeasurementData` struct, `OperationMode` enum |
| `SensorService` | Reads INA226 (power), GPS (NMEA/TinyGPS++), RTC (DS3231), and 4x ADC channels with EMA filtering |
| `TelemetryService` | Logs sensor data to SD Card (CSV) and Serial output; saves captured photos to SD |
| `WebService` | Hosts the web dashboard (SoftAP + STA), live `/json` API, camera stream, and mode switching |
| `MqttService` | Publishes telemetry to HiveMQ; receives remote mode commands |
| `CameraService` | Manages the OV2640 camera module; provides frame access and capture triggering |

---

## Configuration (`DataModel.h`)

### Feature Switches
```cpp
#define ENABLE_WIFI            1
#define ENABLE_CAMERA          1
#define ENABLE_SD              1
#define ENABLE_MQTT            1
#define ENABLE_WIFI_ENTERPRISE 1  // 1 = WPA2 Enterprise (eduroam), 0 = WPA2 Personal
#define ENABLE_MQTT_TLS        1  // 1 = Port 8883 (TLS), 0 = Port 1883 (plain)
#define ENABLE_INA226          1
#define ENABLE_RTC             1
#define ENABLE_GPS             1
```

### WiFi Credentials
```cpp
// WPA2 Personal (home/hotspot)
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// WPA2 Enterprise (eduroam)
#define EAP_IDENTITY "username@university.ac.th"
#define EAP_USERNAME "username@university.ac.th"
#define EAP_PASSWORD "password"
```
> **Tip:** When using eduroam, set `WIFI_SSID "eduroam"` and `ENABLE_WIFI_ENTERPRISE 1`.

### MQTT Configuration
```cpp
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_TOPIC  "cubesat/telemetry"
// MQTT_PORT is automatically 8883 (TLS) or 1883 based on ENABLE_MQTT_TLS
```

---

## WiFi Modes

The ESP32-S3 runs **Dual WiFi (AP + STA) simultaneously**.

| Mode | Details |
| --- | --- |
| **SoftAP** (always on) | SSID: `Cubesat_GROUP4` · Password: `12345678` · IP: `192.168.4.1` |
| **STA Personal** | Connects to home/hotspot WiFi using `WIFI_SSID` + `WIFI_PASS` |
| **STA Enterprise** | Connects to eduroam using EAP Identity/Username/Password (WPA2-EAP) |

---

## Operation Modes

Modes are controlled via the web dashboard, or remotely via MQTT command topic `cubesat/command`.

| Mode | Behavior | MQTT Command |
| --- | --- | --- |
| **SENSOR** *(default)* | Reads all sensors, logs to CSV, publishes to MQTT | `sensor` |
| **CAMERA** | Activates camera stream and capture; sensors paused | `camera` |
| **SLEEP** | All sensor/camera tasks suspended; network still active | `sleep` |

---

## System Workflow

```text
┌─────────────────────────────────────────────────────────────────────┐
│                        CORE 0 (Hardware)                            │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │ SensorTask (Priority 2)                                      │   │
│  │  1. Read INA226, GPS, RTC, 4x ADC (EMA filter)               │   │
│  │  2. Update latest data (Mutex protected)                     │   │
│  │  3. Push MeasurementData to FreeRTOS Queue                   │   │
│  │  4. Sleep 1000ms  →  repeat                                  │   │
│  └─────────────────────┬────────────────────────────────────────┘   │
│                        │ (Queue)                                    │
│  ┌─────────────────────▼────────────────────────────────────────┐   │
│  │ TelemetryTask (Priority 1)                                   │   │
│  │  1. Pop MeasurementData from Queue                           │   │
│  │  2. Write CSV row to /datalog.csv on SD Card                 │   │
│  │     (Timestamp, Mode, INA226, GPS, Satellites, SNR, ADC×4)   │   │
│  │  3. Print /* ... */ to Serial (Serial Studio compatible)     │   │
│  │  4. If capture requested → save /photos/img_DATE_Time_TIME   │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                       CORE 1 (Connectivity)                         │
│                                                                     │
│  Arduino Loop()                                                     │
│   ├── WebService::update()   → handles HTTP clients                 │
│   │    ├── GET /             → Dashboard HTML                       │
│   │    ├── GET /json         → Live sensor JSON                     │
│   │    ├── GET /jpg          → Camera JPEG frame                    │
│   │    ├── GET /capture      → Trigger photo to SD                  │
│   │    └── GET /setMode?m=   → Change operation mode                │
│   │                                                                 │
│   └── MqttService::update()  → keep-alive + publish                 │
│        ├── Publish to cubesat/telemetry  (every 5s)                 │
│        └── Subscribe cubesat/command    (mode control)              │
└─────────────────────────────────────────────────────────────────────┘

         SoftAP ──────────────────────────────► Browser (192.168.4.1)
         STA (Personal/Enterprise) ──► HiveMQ ──► Node-RED Dashboard
```

---

## Data Path

```
Sensors (INA226 / GPS / RTC / ADC)
         │
         ▼
   [ SensorService ]
    ├── Mutex → WebService / MqttService  (pull via getLatestData())
    └── Queue → TelemetryService
                 ├── SD Card: /datalog.csv
                 └── SD Card: /photos/img_YYYY-MM-DD_Time_HH-MM-SS_N.jpg
```

### CSV Columns
```
Timestamp, Mode, Vin(V), Iin(A), Pin(W), Vout(V), Iout(A), Pout(W),
Efficiency(%), Latitude, Longitude, Satellites, SNR, ADC0, ADC1, ADC2, ADC3
```

---

## How to Use

### Step 1 — Configure `DataModel.h`
1. Set WiFi mode (`ENABLE_WIFI_ENTERPRISE`)
2. Fill in credentials (`WIFI_SSID` / `EAP_*` fields)
3. Enable hardware (`ENABLE_INA226`, `ENABLE_GPS`, `ENABLE_RTC`) that is physically connected
4. Set `ENABLE_MQTT_TLS 1` when on eduroam (university blocks port 1883)

### Step 2 — Flash to ESP32-S3
1. Open project in **Arduino IDE**
2. Select board: **ESP32S3 Dev Module**
3. Enable **PSRAM**: `OPI PSRAM` (required for camera)
4. Set **Partition Scheme**: `Huge APP` (if code size error occurs)
5. Upload

### Step 3 — Local Access via SoftAP
1. Connect phone/PC to WiFi `Cubesat_GROUP4` (password: `12345678`)
2. Open browser → `http://192.168.4.1`
3. Dashboard shows: WiFi status, MQTT status, GPS, Battery, Mode, Last Photo filename
4. Use buttons to switch mode or trigger a photo capture

### Step 4 — Remote Monitoring via Node-RED
1. Ensure MQTT is connected (check Serial Monitor at 115200 baud)
2. Import `datasheet esp32-s3 Pinout/nodered_flow.json` into Node-RED
3. Data streams to MQTT broker → Node-RED displays live charts
4. Send commands from Node-RED MQTT inject node to `cubesat/command`:
   - `sensor` → Sensor mode
   - `camera` → Camera mode
   - `sleep` → Sleep mode

### Step 5 — Reading SD Card Data
- Remove SD card and open `/datalog.csv` in Excel or any CSV viewer
- Photos are in `/photos/` named as `img_2026-03-07_Time_22-30-01_0.jpg`

---

## FreeRTOS Task Summary

| Task | Priority | Core | Stack | Interval |
| --- | --- | --- | --- | --- |
| `SensorTask` | 2 (High) | 0 | 4096 | 1000 ms |
| `TelemetryTask` | 1 (Low) | 0 | 4096 | Queue-driven |
| `Arduino Loop` (Web + MQTT) | 1 (Low) | 1 | System | 10 ms |

---

## Pin Reference

| Peripheral | Pins |
| --- | --- |
| I2C (INA226, RTC) | SDA=41, SCL=42 |
| GPS (UART) | RX=21, TX=47 |
| ADC (socket order) | ADC0=Pin37, ADC1=Pin38, ADC2=Pin39, ADC3=Pin40 |
| SD Card (SD_MMC 1-bit) | CLK=39, CMD=38, D0=40 |
| Camera | XCLK=15, SIOD=4, SIOC=5, Y2-Y9=11,9,8,10,12,18,17,16, VSYNC=6, HREF=7, PCLK=13 |
