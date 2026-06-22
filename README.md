# DriveSense — Driver-Behaviour & Collision Detection on STM32L475

On-device machine learning for driving telematics. An **STM32L475** (B-L475E-IOT01A
IoT Discovery node) reads its onboard 6-axis IMU, runs a **TensorFlow Lite Micro**
neural network to classify how the vehicle is being driven, detects **collisions**
in hardware, logs everything to an **SD card**, and uploads it over **Wi-Fi** to
**ThingSpeak**.

```
 LSM6DSL IMU ─► sliding window ─► 48 features ─► TFLite MLP ─► class (0–5)
   (I2C)          (28×6, 2 Hz)                   (TFLM)        + confidence
      │                                                            │
      ├─► hardware wake-up interrupt ─────────────► COLLISION flag │
      │                                                            ▼
 GPS (UART4) ─► lat/lon/speed/UTC time ──────────►  SD card (JSON log, SPI1)
                                                            │
                                                  Wi-Fi (ISM43362, SPI3)
                                                            ▼
                                                   ThingSpeak (HTTP)
```

> **Related repositories**
> - **Mobile app** — the Flutter app that visualises trips, routes, driver scores
>   and collision alerts:
>   [**AI-Drive-Metrics-App**](https://github.com/Muhammad-Miqdad-Ahmad/AI-Drive-Metrices-App).
> - **ML models** — training, datasets and model export that produce the
>   `.tflite` deployed here:
>   [**DriveMetrcisAI-Models**](https://github.com/maheen-zahid-26/DriveMetrcisAI-Models).

---

## Table of contents

1. [Hardware](#1-hardware)
2. [Pinout / wiring](#2-pinout--wiring)
3. [How the firmware works](#3-how-the-firmware-works)
4. [The ML model & TFLite Micro integration](#4-the-ml-model--tflite-micro-integration)
5. [External libraries & the build system](#5-external-libraries--the-build-system)
6. [Building & flashing](#6-building--flashing)
7. [Serial output](#7-serial-output)
8. [SD-card logging](#8-sd-card-logging)
9. [Wi-Fi & ThingSpeak upload](#9-wi-fi--thingspeak-upload)
10. [Collision detection](#10-collision-detection)
11. [Project file map](#11-project-file-map)
12. [Configuration knobs](#12-configuration-knobs)
13. [Known limitations](#13-known-limitations)
14. [Further reading](#14-further-reading)

---

## 1. Hardware

| Part | Role | Bus | Notes |
|------|------|-----|-------|
| **STM32L475VG** (B-L475E-IOT01A) | MCU | — | Cortex-M4F @ 80 MHz, 1 MB Flash, 128 KB SRAM (96 KB SRAM1 + 32 KB SRAM2) |
| **LSM6DSL** (onboard) | 6-axis IMU (accel + gyro) | I2C2 (addr `0xD5`) | The classifier's only sensor input |
| **ISM43362-M3G-L44** (onboard) | Wi-Fi (es-WiFi) | SPI3 | 2.4 GHz 802.11 b/g/n; used for ThingSpeak upload |
| **Micro-SD card module** (external) | Data logging | SPI1 | Module has its own 3.3 V regulator → needs **5 V** in |
| **GPS module** (external, NMEA) | Location, speed, UTC time | UART4 @ 9600 | `$GPRMC` / `$GNRMC` parsed |

The SD card and GPS are the only external add-ons; the IMU and Wi-Fi are built onto
the Discovery board.

---

## 2. Pinout / wiring

### SD-card module → STM32 (SPI1)

| SD module pin | STM32 pin | Arduino label | Wire colour | Notes |
|---------------|-----------|---------------|-------------|-------|
| VCC | 5V  | 5V  | Red    | Module has an onboard 3.3 V regulator — feed it **5 V** |
| GND | GND | GND | Brown  | — |
| SCK | PA5 | D13 | Purple | `SPI1_SCK` |
| MISO| PA6 | D12 | Yellow | `SPI1_MISO` |
| MOSI| PA7 | D11 | Orange | `SPI1_MOSI` |
| CS  | PA2 | D10 | Blue   | GPIO output, active-low (driven high at boot) |

### GPS module → STM32 (UART4)

| GPS module pin | STM32 pin | Arduino label | Wire colour | Notes |
|----------------|-----------|---------------|-------------|-------|
| VCC | 3.3V | 3.3V | White | — |
| GND | GND  | GND  | Black | — |
| TXD | PA1  | D0   | Green | → STM32 `UART4_RX` |
| RXD | PA0  | D1   | Yellow| ← STM32 `UART4_TX` |

> GPS was moved off UART2 (PD5/PD6, not broken out on this board) to **UART4 on
> PA0/PA1** (Arduino header CN3). UART4 RX runs interrupt-driven with overrun (ORE)
> recovery — see [`gps.c`](Core/Src/gps.c).

### Onboard peripherals (fixed by the board)

| Function | STM32 pins |
|----------|------------|
| Debug UART (`printf`) | USART1 — PB6/PB7 (ST-Link virtual COM) @ **115200** |
| LSM6DSL IMU | I2C2 + **INT1 = PD11** (used for the collision wake-up interrupt) |
| ISM43362 Wi-Fi | SPI3 + DRDY = PE1 |
| User LED (LED2, green) | PB14 — blinks on each SD write / collision |

---

## 3. How the firmware works

Entry point is [`Core/Src/main.cpp`](Core/Src/main.cpp) (compiled instead of the
CubeMX-generated `main.c`). The main loop:

1. **Samples the IMU at 2 Hz** (every 500 ms via `HAL_GetTick()`). The model was
   trained at this rate; the LSM6DSL's lowest ODR is 12.5 Hz, so we poll the latest
   reading. Each sample is `[GyroX, GyroY, GyroZ, AccX, AccY, AccZ]` in dps / g.
2. **Feeds the sample into the classifier** ([`classifier.cpp`](Core/Src/classifier.cpp)),
   which maintains a **28-sample × 6-channel sliding window** with 50 % overlap
   (inference fires every 14 new samples).
3. On a completed window it **extracts 48 statistical features** (8 stats × 6 axes:
   mean, std, min, max, Q1, Q3, RMS, range), **mean-centres the gyro axes** to remove
   per-sensor DC bias, **normalises** with the StandardScaler constants, and runs
   **TFLite Micro inference** → one of 6 classes + probabilities.
4. **Reads GPS** ([`gps.c`](Core/Src/gps.c)) for lat/lon/speed and a UTC time anchor.
5. **Logs** the prediction + full window + GPS + harshness to the SD card
   ([`sd_log.c`](Core/Src/sd_log.c)) and blinks LED2.
6. **Uploads** the whole SD log to ThingSpeak ([`thingspeak.c`](Core/Src/thingspeak.c))
   when the GPS fix enters the **home geofence** (`HOME_LAT`/`HOME_LON`, see
   [§9](#9-wi-fi--thingspeak-upload)). A detected collision also forces an immediate
   upload, regardless of location.

In parallel, the LSM6DSL watches for **collisions in hardware** and interrupts the
MCU on impact (see [§10](#10-collision-detection)).

### The 6 classes

`0 = Idle State`, `1 = Normal Driving`, `2 = Sudden Acceleration`,
`3 = Sudden Right Turn`, `4 = Sudden Left Turn`, `5 = Sudden Brake`
(defined in [`classifier.cpp`](Core/Src/classifier.cpp)).

---

## 4. The ML model & TFLite Micro integration

### Architecture & training

A Keras MLP — `Input(48) → Dense(64, ReLU) → Dense(32, ReLU) → Dense(6, Softmax)`,
~5.4 k parameters, ~98.7 % accuracy. Trained in
[`driver_behaviour_detection.ipynb`](driver_behaviour_detection.ipynb) on statistical
features of IMU windows. A `StandardScaler` is fitted on the training set; its
`mean_` and `scale_` arrays are copied into `classifier.cpp` as `SCALER_MEAN` /
`SCALER_SCALE`.

> **Gyro bias fix:** both the notebook's `window_features()` and the firmware
> mean-centre the three gyro axes per window before computing features. The
> gyroscope measures *rate*, not tilt, and different sensor units carry different
> DC offsets; centring removes that offset so the model generalises across sensors.
> After centring, the gyro-mean features have zero variance (their `SCALER_SCALE`
> is 0) and are fed as 0 with a divide-by-zero guard. Accelerometer axes keep their
> DC component (gravity). See [`MODEL_SAMPLING_RATE_FIX.md`](MODEL_SAMPLING_RATE_FIX.md)
> and the notebook.

### Export — float32, **no quantisation**

```python
converter = tf.lite.TFLiteConverter.from_keras_model(mlp_model)
tflite_model = converter.convert()          # float32 weights & activations
open('driver_behaviour_mlp.tflite', 'wb').write(tflite_model)
```

**Do not set `converter.optimizations`.** This TFLM build only supports
`float32 × float32` or `int8 × int8` in its FullyConnected kernel. A hybrid model
(float32 activations + int8 weights) reads int8 bytes as float and silently outputs
NaN.

### Embedding the model in firmware

The `.tflite` file is turned into a C byte array and compiled into Flash. Re-run this
from the project root whenever the model changes:

```bash
echo '#include "model_tflite.h"' > Core/Src/model_tflite.cc && \
xxd -i driver_behaviour_mlp.tflite \
  | sed '1s/.*/const unsigned char model_tflite[] = {/' \
  | sed '/^unsigned int/d' \
  >> Core/Src/model_tflite.cc
```

[`Core/Inc/model_tflite.h`](Core/Inc/model_tflite.h) just declares
`extern const unsigned char model_tflite[];`.

### Runtime (in [`classifier.cpp`](Core/Src/classifier.cpp))

```cpp
tflite::InitializeTarget();
const tflite::Model *model = tflite::GetModel(model_tflite);
static tflite::MicroMutableOpResolver<3> resolver;   // only the ops this model uses
resolver.AddFullyConnected();
resolver.AddRelu();
resolver.AddSoftmax();
static tflite::MicroInterpreter interp(model, resolver, tensor_arena, kTensorArenaSize);
interp.AllocateTensors();
```

The tensor arena is a static `48000`-byte buffer (`alignas(16)`). We use **TFLite
Micro directly** (`Lib/libtflm.a`), *not* ST's X-CUBE-AI / Cube.AI — `xxd` embeds the
flatbuffer and TFLM loads it at runtime.

---

## 5. External libraries & the build system

### Pre-built static libraries (`Lib/`)

| Library | Provides | Built from |
|---------|----------|-----------|
| `Lib/libtflm.a` | TensorFlow Lite Micro runtime (interpreter, kernels) | `tflite-micro/` |
| `Lib/libcmsis-nn.a` | ARM CMSIS-NN optimised NN kernels (int8 paths) | `CMSIS-NN/` |
| `Lib/libCMSISDSP.a` | ARM CMSIS-DSP math routines | `CMSIS-DSP/` |

These are linked in [`CMakeLists.txt`](CMakeLists.txt):

```cmake
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Lib/libtflm.a
    ${CMAKE_SOURCE_DIR}/Lib/libcmsis-nn.a   # MUST come after libtflm.a
    ${CMAKE_SOURCE_DIR}/Lib/libCMSISDSP.a
)
```

> **Link order matters.** GNU `ld` scans archives left-to-right. `libtflm.a` creates
> the undefined references (e.g. `arm_fully_connected_s8`) that `libcmsis-nn.a`
> satisfies, so tflm must be listed **first**, or the linker won't rescan and you get
> `undefined reference` errors.

### Header include paths (in `CMakeLists.txt`)

The TFLM headers and its bundled third-party deps (flatbuffers, kissfft, ruy,
gemmlowp) plus CMSIS-DSP/NN headers are added under
`target_include_directories(...)`.

### TFLM support shims (added to `target_sources`)

| File | Why |
|------|-----|
| [`Core/Src/debug_log.cc`](Core/Src/debug_log.cc) | Implements `DebugLog` so TFLM error messages reach UART |
| [`Core/Src/micro_time.cc`](Core/Src/micro_time.cc) | Implements TFLM's timing hooks |
| [`Core/Src/model_tflite.cc`](Core/Src/model_tflite.cc) | The embedded model byte array |

### CubeMX layer (`cmake/stm32cubemx/`)

CubeMX-generated sources (HAL drivers, FatFS, Wi-Fi driver, BSP, the application
sources) are collected in [`cmake/stm32cubemx/CMakeLists.txt`](cmake/stm32cubemx/CMakeLists.txt)
and exposed as the `stm32cubemx` target. **Our application `.c`/`.cpp` files
(`main.cpp`, `classifier.cpp`, `gps.c`, `sd_log.c`, `thingspeak.c`, FatFS, Wi-Fi)
are listed in `MX_Application_Src` there** — if CubeMX regenerates code it tends to
revert `main.cpp` back to `main.c` and drop these, so they must be re-added (see
[`SD_CARD_INTEGRATION.md`](SD_CARD_INTEGRATION.md)).

### Toolchain & key flags

- **Toolchain:** `arm-none-eabi-gcc`, via `cmake/gcc-arm-none-eabi.cmake`.
- **Generator:** Ninja, configured through [`CMakePresets.json`](CMakePresets.json)
  (`Debug` / `Release`).
- **C11 / C++17**, `USE_HAL_DRIVER`, `STM32L475xx` defined.
- **`-u _printf_float`** added to the linker flags so newlib-nano's `printf` handles
  `%f`. (newlib-nano also has **no `%llu`** — that's why timestamps are sent as
  32-bit `%lu`; see [`thingspeak.c`](Core/Src/thingspeak.c).)
- **Stack raised to 8 KB** (`_Min_Stack_Size = 0x2000`) in
  [`STM32L475XX_FLASH.ld`](STM32L475XX_FLASH.ld) for the TFLM + `printf` call depth.

---

## 6. Building & flashing

```bash
# Configure (first time, or after CubeMX regen / CMakeLists change)
cmake --preset Debug

# Build
cmake --build build/Debug
# → build/Debug/upload_test_2.elf
```

Flash with **STM32CubeProgrammer**:

```bash
STM32_Programmer_CLI -c port=SWD -w build/Debug/upload_test_2.elf -v -rst
```

### Memory footprint

From the linker's report on the current `Debug` build (the whole firmware —
TFLite Micro model + IMU + GPS + SD/FatFS + Wi-Fi/ThingSpeak + collision detection):

| Region | Used | Available | % used |
|--------|------|-----------|--------|
| **RAM** (SRAM1) | 63,536 B (≈62 KB) | 96 KB | **64.6 %** |
| **RAM2** (SRAM2) | 0 B | 32 KB | 0 % |
| **FLASH** | 192,168 B (≈188 KB) | 1 MB | **18.3 %** |

Plenty of headroom on both — most of RAM is the 48 KB TFLite tensor arena plus
the 8 KB stack, and most of Flash is the embedded model and the HAL/Wi-Fi/FatFS
drivers. (The `arm-none-eabi` build prints these figures at the end of every
link.)

---

## 7. Serial output

Debug `printf` goes out **USART1 at 115200 baud** over the ST-Link virtual COM port.
`__io_putchar` (in `main.cpp`) sends `\r` before `\n` so terminals render cleanly.

```bash
picocom -b 115200 /dev/ttyACM0
```

Per prediction you'll see the class + confidence, all 6 probabilities, the GPS fix,
the full 28-sample IMU window, and — on impact — `*** COLLISION detected: ~X.XX g ***`.

---

## 8. SD-card logging

[`sd_log.c`](Core/Src/sd_log.c) appends one JSON object per prediction to
`0:/data.log` via FatFS over SPI1. The log is **truncated on every boot**
(`FA_CREATE_ALWAYS`) so a single file only ever holds one power session, and it is
also cleared after a **fully successful** ThingSpeak upload. (One-session-per-file
avoids ThingSpeak's `error_duplicate_timestamps`: the `time` field is
`HAL_GetTick()`, which restarts at 0 each boot, so mixing sessions would map
different runs to the same wall-clock second — see [§9](#9-wi-fi--thingspeak-upload).)

```json
{"time":512000,"pred":1,"label":"Normal Driving","conf":92.4,
 "lat":31.515000,"lon":74.465000,"spd":18.3,"fix":1,
 "gmax":1.07,"collision":0,"gpeak":0.00,
 "gx":[...28...],"gy":[...],"gz":[...],"ax":[...],"ay":[...],"az":[...]}
```

- `gmax` — peak |a| over the window (harshness).
- `collision` / `gpeak` — set to `1` / impact peak-g on the first record after a crash.
- `gx…az` — the **entire 28-sample window** (raw) the model classified.

Read it back in Python:

```python
import json
rows = [json.loads(l) for l in open("data.log") if l.strip()]
```

### SD / FatFS notes

The SPI SD driver lives in [`FATFS/Target/user_diskio.c`](FATFS/Target/user_diskio.c)
(CubeMX generates only a stub — it was implemented by hand: CMD0/8/41/58 init,
≤400 kHz init clock via prescaler 256, CS held high at boot, auto-`f_mkfs` on mount
failure). **LFN is disabled**, so 8.3 filenames only (hence `data.log`, not
`log.json`). Full debugging story in
[`SD_CARD_INTEGRATION.md`](SD_CARD_INTEGRATION.md).

---

## 9. Wi-Fi & ThingSpeak upload

[`thingspeak.c`](Core/Src/thingspeak.c) joins Wi-Fi (es-WiFi / ISM43362 on SPI3),
resolves `api.thingspeak.com` (with DNS retries — the module's resolver is flaky),
and POSTs the SD log as **bulk updates** over plain HTTP. The free tier allows one
bulk request per 15 s, so a large backlog is sent in batches (~18 events each, with
all 8 fields populated) with a retry on failure.

> **Alternative transport — `mosquitto` branch.** This `main` branch uploads to
> ThingSpeak. A separate **[`mosquitto`](../../tree/mosquitto)** branch replaces that
> with a self-hosted path: the board publishes over **MQTT** to a **Mosquitto** broker
> on a local PC, and a Python bridge (`tools/mqtt_bridge.py`) forwards the data
> straight into Supabase. It drops the cloud dependency (plain TCP on the LAN — no
> TLS/SNI, no rate limits) and is the basis for future work. See that branch's
> `mqtt.c` / `mqtt.h` and `tools/mqtt_bridge.py`.

### When does it upload?

There are two triggers — there is **no fixed every-N-predictions cadence**
(`UPLOAD_EVERY_N` is defined in `net_config.h` but currently unused):

- **Home geofence (normal case).** Each prediction calls `ThingSpeak_Process(gps)`,
  which uploads and clears the whole log the moment the GPS fix first enters a circle
  of radius `HOME_RADIUS_M` around `HOME_LAT`/`HOME_LON`. Hysteresis (must travel
  > 1.5× the radius away before re-arming) stops it re-uploading while parked, and the
  geofence only disarms on a **successful** upload, so a failed attempt retries on the
  next pass.
- **Collision (immediate).** A detected impact sets `forceUpload`, which calls
  `ThingSpeak_UploadNow()` on the next prediction so the crash reaches the cloud
  within seconds, wherever the vehicle is.

> **No duplicate timestamps.** ThingSpeak rejects a bulk update containing two equal
> `created_at` values (`error_duplicate_timestamps`). The uploader forces strictly
> increasing per-record seconds (bumping to `prev + 1` on a collision/tie), and the
> clear-on-boot log (see [§8](#8-sd-card-logging)) keeps each file to one session.

### ThingSpeak field mapping

| Field | Meaning |
|-------|---------|
| field1 | Prediction class (0–5) |
| field2 | Confidence (%) |
| field3 | Latitude |
| field4 | Longitude |
| field5 | Speed (km/h) |
| field6 | `Gmax` — peak window g-force (harshness) |
| field7 | Unix epoch seconds (device/GPS timestamp) |
| field8 | `Collision` flag (1 = detected impact) |

Each entry also carries a `created_at` ISO-8601 timestamp.

> **Timestamps need GPS.** The board has no RTC; UTC time comes from the GPS
> `$GPRMC` sentence ([`gps.c`](Core/Src/gps.c) sets a tick↔epoch anchor). With no
> fix, timestamps fall back to seconds-since-boot (≈ 1970), which is expected
> indoors.

### Configuration — [`Core/Inc/net_config.h`](Core/Inc/net_config.h)

Set your Wi-Fi networks (`NET_WIFI_NETWORKS`, tried in order), the ThingSpeak
`THINGSPEAK_CHANNEL_ID` + `THINGSPEAK_WRITE_KEY`, and the home geofence
(`HOME_LAT` / `HOME_LON` / `HOME_RADIUS_M`).

> Earlier the project targeted Supabase, but the ISM43362's TLS **cannot send SNI**,
> which Supabase (behind Cloudflare) requires — so direct HTTPS from the board is
> impossible. ThingSpeak over plain HTTP works directly. (A laptop HTTP→HTTPS relay
> was the Supabase workaround; no longer used.)

---

## 10. Collision detection

The 2 Hz poll loop is far too slow to catch a ~100 ms crash pulse, and ±2 g would
clip on any real impact. So collision detection runs **entirely in the LSM6DSL
hardware**:

- The accelerometer is set to **±8 g** and its **wake-up (over-threshold) interrupt**
  is enabled on **INT1 (PD11)**, watching at the sensor's 416 Hz internal rate.
- Threshold ≈ **3 g** (`COLLISION_WAKEUP_THS = 24`; `WAKE_UP_THS` LSB = FS/64 =
  0.125 g at ±8 g). This sits well above harsh driving (~0.6 g).
- On impact the sensor raises PD11; the ISR sets a flag, and the main loop confirms
  via `Get_Event_Status`, reads the peak g, flashes LED2 in a distinct 6-blink
  pattern, tags the next record `collision:1`, and **forces an immediate ThingSpeak
  upload**.

Tune `COLLISION_WAKEUP_THS` in [`main.cpp`](Core/Src/main.cpp) (each step = 0.125 g)
after bench-testing with a firm tap. The full design rationale (airbag/telematics
g-thresholds, sampling-rate analysis) is in the plan that produced this feature.

---

## 11. Project file map

### Application code — `Core/`

| File | Purpose |
|------|---------|
| [`Core/Src/main.cpp`](Core/Src/main.cpp) | Entry point, peripheral init, main loop, IMU sampling, collision service, UART `printf` |
| [`Core/Src/classifier.cpp`](Core/Src/classifier.cpp) · [`.h`](Core/Inc/classifier.h) | Sliding window, feature extraction, gyro centring, scaler, TFLM inference |
| [`Core/Src/gps.c`](Core/Src/gps.c) · [`gps.h`](Core/Inc/gps.h) | NMEA `$GPRMC` parsing, UTC time anchor, UART4 RX + ORE recovery |
| [`Core/Src/sd_log.c`](Core/Src/sd_log.c) · [`sd_log.h`](Core/Inc/sd_log.h) | JSON logging to SD, pause/resume/clear for upload |
| [`Core/Src/thingspeak.c`](Core/Src/thingspeak.c) · [`thingspeak.h`](Core/Inc/thingspeak.h) | Wi-Fi join, DNS, ThingSpeak bulk HTTP upload |
| [`Core/Inc/net_config.h`](Core/Inc/net_config.h) | Wi-Fi credentials, ThingSpeak channel, upload cadence |
| [`Core/Inc/app_log.h`](Core/Inc/app_log.h) | `DEBUG_PRINTF` / `SUCCESS_PRINTF` macros |
| [`Core/Src/model_tflite.cc`](Core/Src/model_tflite.cc) · [`model_tflite.h`](Core/Inc/model_tflite.h) | Embedded `.tflite` byte array |
| [`Core/Src/debug_log.cc`](Core/Src/debug_log.cc), [`micro_time.cc`](Core/Src/micro_time.cc) | TFLM platform shims |
| `Core/Src/main.c` | CubeMX-generated original (**not** compiled — `main.cpp` is used instead) |
| `Core/Src/stm32l4xx_it.c`, `stm32l4xx_hal_msp.c`, `syscalls.c`, `sysmem.c`, `system_stm32l4xx.c` | CubeMX/HAL plumbing (UART4 IRQ, EXTI, etc.) |

### Middleware, drivers, libraries

| Path | Contents |
|------|----------|
| `FATFS/` | FatFS app glue + the hand-written SPI SD driver (`Target/user_diskio.c`) |
| `Middlewares/Third_Party/FatFs/` | FatFS core (untouched) |
| `Drivers/BSP/Components/lsm6dsl/` | ST LSM6DSL driver (wake-up, full-scale, ODR APIs) |
| `Drivers/WiFi/` | Inventek es-WiFi (ISM43362) driver |
| `Drivers/STM32L4xx_HAL_Driver/`, `Drivers/CMSIS/` | ST HAL + CMSIS |
| `Lib/` | `libtflm.a`, `libcmsis-nn.a`, `libCMSISDSP.a` |
| `tflite-micro/`, `CMSIS-NN/`, `CMSIS-DSP/` | Sources the `Lib/*.a` were built from + headers used at compile time |
| `X-CUBE-MEMS1/` | ST MEMS BSP layer |

### Build & config

| Path | Purpose |
|------|---------|
| [`CMakeLists.txt`](CMakeLists.txt) | Top-level build: sources, includes, library link |
| [`cmake/stm32cubemx/CMakeLists.txt`](cmake/stm32cubemx/CMakeLists.txt) | CubeMX-generated source/include lists |
| `cmake/gcc-arm-none-eabi.cmake` | ARM toolchain file |
| [`CMakePresets.json`](CMakePresets.json) | `Debug` / `Release` presets (Ninja) |
| [`STM32L475XX_FLASH.ld`](STM32L475XX_FLASH.ld) | Linker script (8 KB stack) |
| `upload_test_2.ioc` | CubeMX project (peripheral config) |

### ML & docs

| Path | Purpose |
|------|---------|
| [`driver_behaviour_detection.ipynb`](driver_behaviour_detection.ipynb) | Training, feature engineering, scaler/TFLite export |
| `driver_behaviour_mlp.tflite` | The trained model (source for `model_tflite.cc`) |
| [`HOWTO.md`](HOWTO.md), [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md), [`MODEL_SAMPLING_RATE_FIX.md`](MODEL_SAMPLING_RATE_FIX.md), [`SD_CARD_INTEGRATION.md`](SD_CARD_INTEGRATION.md) | Build / model / SD-card deep dives |

---

## 12. Configuration knobs

| Setting | Where | Default |
|---------|-------|---------|
| Wi-Fi networks | `net_config.h` → `NET_WIFI_NETWORKS` | (fill in) |
| ThingSpeak channel + write key | `net_config.h` | (fill in) |
| Upload trigger | `net_config.h` → `HOME_LAT`/`HOME_LON`/`HOME_RADIUS_M` | home geofence + collision (no fixed cadence; `UPLOAD_EVERY_N` is unused) |
| Collision threshold | `main.cpp` → `COLLISION_WAKEUP_THS` | 24 (≈ 3 g @ ±8 g) |
| IMU sample period | `main.cpp` main loop | 500 ms (2 Hz) |
| Window / overlap | `classifier.cpp` | 28 samples, 50 % |
| Tensor arena | `classifier.cpp` → `kTensorArenaSize` | 48 KB |
| Debug baud | `main.cpp` `MX_USART1_UART_Init` | 115200 |

---

## 13. Known limitations

- **Collision severity isn't measured.** ±8 g clips above 8 g; a real cabin crash is
  30–60 g, so we reliably *detect* an impact but `gpeak` saturates. True severity
  would need a ±100 g-class high-g sensor (e.g. ST AIS2120SX / LSM6DSV320X).
- **Timestamps require a GPS fix** (no RTC) — indoors they read ≈ 1970.
- **SD `f_sync` runs every 10 writes**, so a power cut can lose up to ~9 records, and
  pulling the card mid-write can corrupt the last line (no card-detect pin).
- **The log clears on every boot.** A mid-trip reset (before reaching the home
  geofence) discards that session's data. This is the deliberate fix for ThingSpeak's
  duplicate-timestamp rejection; for cross-reboot persistence the clear-on-boot in
  [`sd_log.c`](Core/Src/sd_log.c) would need to be dropped (the uploader's
  strictly-increasing-timestamp guard alone still prevents the 400).
- **Consumer-grade**, not automotive-qualified — this is a telematics/learning
  project, not a safety system.
- **CubeMX regeneration** reverts `main.cpp`→`main.c` and drops our sources from the
  CubeMX CMake list; re-apply them after any regen.

---

## 14. Further reading

- [`HOWTO.md`](HOWTO.md) — original TFLite-on-STM32 build walk-through
- [`IMPLEMENTATION_NOTES.md`](IMPLEMENTATION_NOTES.md) — model details, `xxd` embedding, troubleshooting table
- [`MODEL_SAMPLING_RATE_FIX.md`](MODEL_SAMPLING_RATE_FIX.md) — sampling-rate & gyro-bias fixes
- [`SD_CARD_INTEGRATION.md`](SD_CARD_INTEGRATION.md) — SD wiring & the bring-up saga
- Datasheets in repo root: `stm32l475vg.pdf`, `STM32L475-Reference_Manual.pdf`, `stm32-IOT_NODE_Manual.pdf`

---

## 15. License & citation

The **application code in this project** (the files under `Core/Src` / `Core/Inc`
authored here, and `tools/`) is released under the **MIT License** — see
[`LICENSE`](LICENSE). You're free to use, modify, and redistribute it with
attribution.

> **Third-party components keep their own licenses.** The vendored libraries are
> *not* MIT and retain their original terms — `tflite-micro/`, `CMSIS-NN/`,
> `CMSIS-DSP/` (Apache-2.0), the STM32 HAL & CMSIS device files (BSD-3-Clause /
> ST license), FatFS (BSD-style), and the Inventek es-WiFi driver (`Drivers/WiFi/`,
> ST/Inventek license). Their `LICENSE`/headers are preserved in their folders.

If you use this work in research, please cite it via
[`CITATION.cff`](CITATION.cff) (GitHub shows a **"Cite this repository"** button).
