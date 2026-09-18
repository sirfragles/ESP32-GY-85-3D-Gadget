# ESP32-GY-85-3D-Gadget

[![GitHub stars](https://img.shields.io/github/stars/dominikn/ESP32-MPU9250-web-view?style=social)](https://github.com/DominikN/ESP32-MPU9250-web-view/stargazers/)

[![Build firmware](https://github.com/sirfragles/ESP32-GY-85-3D-Gadget/actions/workflows/build.yml/badge.svg)](https://github.com/sirfragles/ESP32-GY-85-3D-Gadget/actions/workflows/build.yml)
[![GitHub license](https://img.shields.io/github/license/dominikn/ESP32-MPU9250-web-view.svg)](https://github.com/dominikn/ESP32-MPU9250-web-view/blob/master/LICENSE)

**_A 9-DoF orientation gadget: a **Seeed Studio XIAO ESP32C3** reads a **GY-85** module (ADXL345 accelerometer + ITG3205 gyroscope + QMC5883L magnetometer), fuses the data on-chip and streams the orientation to a web page with a 3D cube and roll/pitch/yaw gauges. The page is served by the ESP32 itself over Wi-Fi._**

- **9-DoF sensor fusion on the ESP32** — Kalman-filtered roll/pitch plus a tilt-compensated compass yaw, the same scheme as the [esp-idf-gy85](https://github.com/nopnop2002/esp-idf-gy85) reference demo; no IMU libraries are needed
- **web UI** — Three.js cube and canvas-gauges, fed by a JSON WebSocket stream, with automatic reconnect
- **calibration built in** — at boot, automatically while the module lies flat and still, or manually with the **Calibrate** button; kept in RTC memory and in flash (NVS)
- **motion-activated deep sleep** — after 60 s without movement the board goes to deep sleep; the accelerometer's activity interrupt wakes it up when the module is picked up
- **network friendly** — DHCP + mDNS: just open **`http://esp32c3.local`** (or the IP printed on the serial monitor) on the same network

## Hardware

- **Seeed Studio XIAO ESP32C3** (any ESP32-C3 board works; the pin names below refer to the XIAO)
- **GY-85** module (ITG3205 + ADXL345 + QMC5883L)
- 5 jumper wires; the module can be powered from the XIAO's 3V3 pin

### Wiring and pin description

All three sensors of the GY-85 sit on one I²C bus; a single extra wire is needed for the deep-sleep motion wake-up:

```
XIAO ESP32C3 <-> GY-85

3V3          <-> VCC (3.3 V)
GND          <-> GND
D4 (GPIO6)   <-> SDA
D5 (GPIO7)   <-> SCL
D3 (GPIO5)   <-> A_INT1
```

| XIAO pin | GPIO | Function | GY-85 pin | Notes |
|----------|------|----------|-----------|-------|
| `3V3` | — | 3.3 V power | `VCC` | powers the module |
| `GND` | — | ground | `GND` | common ground |
| `D4` | GPIO6 | I²C data (SDA) | `SDA` | shared by all three sensors |
| `D5` | GPIO7 | I²C clock (SCL) | `SCL` | 100 kHz bus |
| `D3` | GPIO5 | wake-up input | `A_INT1` | ADXL345 interrupt 1 (active high) |

Notes:

- **D3/GPIO5** is one of the ESP32-C3 pins (GPIO0–GPIO5) that can wake the chip from deep sleep — this wire is what makes the motion wake-up (and therefore the automatic sleep) possible
- the remaining GY-85 interrupt outputs are **not used** — leave them unconnected:

| GY-85 pin | Signal |
|-----------|--------|
| `M_DRDY` | magnetometer (QMC5883L) data ready |
| `G_INT` | gyroscope (ITG3205) interrupt |

- these are the default pin assignments; every one of them can be changed at the top of **`gy85_imu.h`** (`GY85_PIN_SDA`, `GY85_PIN_SCL`, `GY85_PIN_WAKE`)

## Files

| File | Purpose |
|------|---------|
| `ESP32-GY-85-3D-Gadget.ino` | main sketch: Wi-Fi, HTTP + WebSocket servers, tasks, deep sleep |
| `gy85_imu.h` / `gy85_imu.cpp` | the IMU module (sketch tabs): GY-85 driver, sensor fusion, calibration, wake-up support — all IMU settings (`GY85_*`) live in the header |
| `index.html` | web page source (cube, gauges, Calibrate button) |
| `index_html.h` | **generated** from `index.html` — do not edit by hand |
| `credentials-template.h` | copy to `credentials.h` (gitignored) and fill in your Wi-Fi credentials |
| `tools/gen_index_html.py` | regenerates `index_html.h` from `index.html` |

## Configuration overview

- in the sketch (`ESP32-GY-85-3D-Gadget.ino`): Wi-Fi credentials (`credentials.h`), `MDNS_HOSTNAME`, `SLEEP_IDLE_MS`
- in `gy85_imu.h`: everything IMU-related — pins, magnetometer offsets and signs, motion thresholds, re-calibration windows (`0` disables the automatic re-calibration), NVS namespace

## Build and flash (Arduino IDE)

**1. Install the ESP32 board support** (skip if you already have it):

- open `File -> Preferences`
- in the field **Additional Board Manager URLs** add: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- open `Tools -> Board -> Boards Manager...`, search for `esp32` by **Espressif Systems** and click **Install**

**2. Select the board:**

- open `Tools -> Board` and select **_XIAO_ESP32C3_**
- set `Tools -> Partition Scheme` to **_Minimal SPIFFS (1.9MB APP with OTA/128KB SPIFFS)_** — the sketch is ~1.20 MB, so the default 1.2 MB scheme barely fits; Minimal SPIFFS leaves comfortable headroom

**3. Install libraries from the Library Manager (`Tools -> Manage Libraries...`):**

- `WebSockets` by Markus Sattler
- `ArduinoJson` in version **6.18.5** (ArduinoJson 7.x is not tested with this sketch)

(the rest — mDNS, NVS, I²C — is part of the ESP32 core; the GY-85 needs **no extra libraries**, it is driven over raw I²C)

**4. Open the sketch and upload:**

- open **ESP32-GY-85-3D-Gadget/ESP32-GY-85-3D-Gadget.ino** — `gy85_imu.h` / `gy85_imu.cpp` are tabs of the same sketch
- copy **credentials-template.h** to **credentials.h** and fill in your Wi-Fi credentials (the file is gitignored)
- upload the project to your board
- on the first boot leave the module **flat and still** for a moment — the firmware calibrates the accelerometer and gyroscope zero offsets and stores them

## Open the web page

- open the serial monitor (115200 baud) to watch the log
- open **`http://esp32c3.local`** (mDNS; macOS/iOS/Windows resolve `.local` names out of the box, some Android browsers need mDNS support), or `http://<IP address>` — the HTTP server listens on **port 80**, so no port has to be typed
- you should see the 3D cube and the Roll/Pitch/Yaw gauges following the module; the page connects its WebSocket to the host name it was opened with, so both ways work

## Web page and data protocol

The ESP32 sends a JSON frame about 50 times per second (only while a browser is connected):

```json
{"q0":w,"q1":x,"q2":y,"q3":z,"roll":deg,"pitch":deg,"yaw":deg}
```

- `q0..q3` — orientation quaternion (w, x, y, z) used for the cube
- `roll` / `pitch` / `yaw` — the same orientation as Euler angles (degrees) used for the gauges

The page can also send text commands back to the board:

- `calibrate` — request a manual zero-offset calibration; the board answers with a status frame:
  `{"cal":"started"}` → sampling began, `{"cal":"done"}` → new offsets applied and stored,
  `{"cal":"refused"}` → the module is not flat and still, `{"cal":"aborted"}` → it moved during sampling; the status is shown next to the button

## Deep sleep (motion activated)

To save power the board switches to **deep sleep** when the module has not moved for a while:

- the trigger is `SLEEP_IDLE_MS` (default **60 s**; `0` disables) **and** no web browser connected — so the cube never freezes while you are watching it
- before sleeping, the gyro and magnetometer are paused and the **ADXL345 activity interrupt** is armed; movement raises `A_INT1`, which wakes the board through D3 (GPIO wake-up)
- on wake the board reboots, reconnects to Wi-Fi (a few seconds) and resumes streaming — **move the module to wake it up**; the web page reconnects its WebSocket automatically
- a re-calibration in progress postpones the sleep, and the idle counter restarts when it finishes
- if the board "looks dead", it is probably sleeping — move the module (or set `SLEEP_IDLE_MS` to `0`)

### Automatic re-calibration while idle

The same "flat and still" detector that runs at boot also works while the firmware is running:

- after `GY85_RECAL_STILL_MS` (default **30 s**) of lying flat and still, the raw accel/gyro data is averaged for `GY85_RECAL_SAMPLE_MS` (default **30 s**), then the new offsets are applied and stored (RTC + NVS) — the measurement is passive, the cube keeps streaming
- once per "still session": the trigger re-arms only after the module is moved; moving during the sampling window aborts it
- with the default timing: 30 s still → 30 s of sampling → the sleep counter restarts → sleep after another 60 s of stillness (when no page is connected)

## Calibration

The accel/gyro zero offsets are measured when the module lies **flat and still** (Z axis carrying +1 g):

| Situation | What happens |
|-----------|--------------|
| wake-up from deep sleep | the offsets kept in **RTC memory** are reused (the module may be moving at wake) |
| boot, module lying flat and still | fresh calibration (200 samples), applied and stored in RTC **and** flash (NVS) |
| boot, module moving or not flat | the calibration stored in NVS is reused — a reset in mid-air cannot produce a wrong calibration |
| **Calibrate** button on the page | ~5 s sampling window (`GY85_RECAL_MANUAL_MS`), accepted only when flat and still |
| automatically while idle | see the section above (`GY85_RECAL_STILL_MS` / `GY85_RECAL_SAMPLE_MS`) |

- to force a fresh calibration: leave the module lying flat and still (it re-calibrates by itself), or power-cycle it while it lies flat
- to forget the stored calibration completely: upload with *Erase All Flash Before Sketch Upload* enabled
- **magnetometer hard-iron offsets**: set `GY85_MAG_OFFSET_X/Y/Z` in `gy85_imu.h`. To find them, leave them at 0, rotate the module through all orientations and note the min/max of each axis from the `mag=[...]` values on the serial monitor — the offset is `(min + max) / 2`
- if the yaw turns the wrong way (or is off by ~180°), flip `GY85_MAG_SIGN_X/Y/Z`
- the driver expects a **QMC5883L** magnetometer (address 0x0D); modules with the older HMC5883L (0x1E) are not supported
- the ITG3205 and ADXL345 addresses are auto-detected (0x68/0x69 and 0x53/0x1D)

## Firmware reference

**IMU module — `gy85_imu.h` / `gy85_imu.cpp`**

- `gy85Init()` — starts the I²C bus, detects and configures the ADXL345, ITG3205 and QMC5883L, then sets the zero offsets (RTC → NVS → fresh, see the table above). Returns `false` when a sensor cannot be reached. Call once from a task.
- `gy85Update(dt)` — reads the sensors and updates the fusion: `gy85Roll`, `gy85Pitch`, `gy85Yaw` (degrees) and the movement flags. Pass the time since the previous call in seconds.
- `gy85RecalTick()` — call once per loop iteration; tracks how long the module has been flat and still, starts the automatic re-calibration and applies the averaged offsets when the window completes. Returns `true` when a new calibration was applied.
- `gy85StartManualRecal()` — starts the short manual calibration window immediately (used by the **Calibrate** button). Returns `false` when the module is moving or not flat.
- `gy85EulerToQuat(roll, pitch, yaw, &w, &x, &y, &z)` — converts Euler angles (degrees) to a quaternion for the 3D cube.
- `gy85PrepareWakeOnMotion()` — pauses the gyro and magnetometer, arms the ADXL345 activity interrupt and configures the wake pin; call it just before entering deep sleep.
- exported state: `gy85Roll` / `gy85Pitch` / `gy85Yaw`, `gy85MagRaw[]`, `gy85Moving`, `gy85RecalActive`, `gy85CalRequest`, `gy85CalReply` (0 = none, 1 = started, 2 = done, 3 = refused, 4 = aborted).

**Application — `ESP32-GY-85-3D-Gadget.ino`**

- `setup()` — starts the serial port, reports a deep-sleep wake and creates both tasks on core 0.
- `taskWifi()` — connects through `WiFiMulti`, starts mDNS (`esp32c3.local`), the HTTP server (port 80) and the WebSocket server (port 8001); reconnects whenever Wi-Fi drops.
- `taskStatus()` — runs the IMU at ~50 Hz: `gy85Update()` → manual calibration request → `gy85RecalTick()` → deep-sleep decision → calibration replies → quaternion + JSON → WebSocket broadcast.
- `onWebSocketEvent()` — connection bookkeeping and command parsing (the `calibrate` text command).
- `onHttpReqFunc()` — serves the embedded web page for `/` and `/index.html`.
- `enterDeepSleep()` — prepares the motion wake-up, turns Wi-Fi off, enables the GPIO wake-up on `GY85_PIN_WAKE` and enters deep sleep (does not return).
- `loop()` — debug only (prints the free heap once per second).

## Editing the web page

The Arduino IDE cannot embed files into a sketch, so the page is compiled in as a C string: `index_html.h` is generated from `index.html`. After every edit of the HTML regenerate the header and re-upload the sketch:

    python3 tools/gen_index_html.py

## Credits and licence

- fork of [DominikN/ESP32-MPU9250-web-view](https://github.com/DominikN/ESP32-MPU9250-web-view) (MIT) — the project structure, the web page idea and the WebSocket/JSON scheme come from there; this fork replaced the IMU side (MPU9250 → GY-85) and added mDNS, the calibration system and the deep-sleep support
- the fusion scheme follows the [nopnop2002/esp-idf-gy85](https://github.com/nopnop2002/esp-idf-gy85) demo
- the gadget works on the local network only — earlier revisions of this fork used Husarnet for remote access, but Husarnet's ESP32 support is archived (esp32 core 1.0.4) and does not work with the ESP32-C3


