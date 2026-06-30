# Implementation Notes — Driver Behaviour MLP on STM32L475 IoT Node

> **Scope:** these notes cover the **initial TFLite Micro bring-up** — deploying the
> model and proving inference on the device with a dummy input. The firmware has since
> gained real IMU sampling, GPS, SD logging, ThingSpeak upload, and hardware collision
> detection; for the full current system see [`README.md`](README.md). Figures below
> have been updated to the current build where they were measured (footprint, baud,
> tensor arena); the historical narrative is kept for context.

## What was implemented

A 6-class driver behaviour classifier (MLP neural network) was deployed on the
STM32L475VGTx microcontroller using TensorFlow Lite Micro. This bring-up first ran
inference on **dummy data** over UART to prove the pipeline; the firmware now feeds
the model **real IMU windows** ([`classifier.cpp`](Core/Src/classifier.cpp)) — see
[`MODEL_SAMPLING_RATE_FIX.md`](MODEL_SAMPLING_RATE_FIX.md).

---

## Model details

| Property | Value |
|----------|-------|
| Architecture | MLP — Input(48) → Dense(64, ReLU) → Dense(32, ReLU) → Dense(6, Softmax) |
| Parameters / accuracy | See the [DriveMetrcisAI-Models](https://github.com/maheen-zahid-26/DriveMetrcisAI-Models) repo (the model has been retrained since these notes) |
| TFLite file | `driver_behaviour_mlp.tflite` (24,008 bytes) |
| Input | 48 normalised statistical features from a 28-sample IMU window |
| Output | 6 class probabilities |
| Classes | Idle State, Normal Driving, Sudden Acceleration, Sudden Right Turn, Sudden Left Turn, Sudden Brake |

---

## Files changed

### Created
| File | Purpose |
|------|---------|
| `Core/Src/model_tflite.cc` | The `.tflite` binary embedded as a C byte array for Flash storage. Generated with `xxd -i driver_behaviour_mlp.tflite`, then renamed to `model_tflite[]` with `const` added. |

### Modified
| File | What changed |
|------|-------------|
| `CMakeLists.txt` | Added `Core/Src/debug_log.cc`, `Core/Src/micro_time.cc`, and `Core/Src/model_tflite.cc` to `target_sources`. These were missing from the build despite being in the project. |
| `Core/Src/main.cpp` | See below. |

#### Changes to `main.cpp`
- **Removed** `waveform[16128]` and `last_ffts[125]` — leftover audio model buffers that wasted 16+ KB of RAM.
- **Reduced** `kTensorArenaSize` from 66,800 → **48,000** bytes. The audio model needed 65 KB for FFT scratch space; this MLP needs far less, and 48 KB leaves a comfortable margin. (Now defined in [`classifier.cpp`](Core/Src/classifier.cpp).)
- **Added** `SCALER_MEAN[48]` and `SCALER_SCALE[48]` — StandardScaler constants extracted from the training notebook. These are applied to features before inference to match training normalisation. (Now in `classifier.cpp`, alongside per-window gyro mean-centring.)
- **Added** `CLASS_NAMES[6]` — human-readable labels for the 6 output classes.
- **Added TFLite initialisation** (now in `classifier.cpp`): model loading, op resolver with just `AddFullyConnected / AddRelu / AddSoftmax` (the float32 model uses no quantize/dequantize ops), interpreter creation, and `AllocateTensors`.
- **Added inference loop**: during bring-up a dummy all-zero feature vector every 2 s; now a real 28×6 IMU sliding window at 2 Hz, `Invoke()`, argmax, and UART print of the prediction + all 6 probabilities.

---

## Why xxd and not STM32Cube.AI?

This project uses **TFLite Micro** (`Lib/libtflm.a`) directly — the open-source TensorFlow runtime for microcontrollers. STM32Cube.AI (X-CUBE-AI) is a separate ST-proprietary tool that generates its own inference code. Switching to Cube.AI would require a completely different integration. Using `xxd -i` embeds the `.tflite` file as a C array that TFLite Micro loads at runtime.

---

## Build instructions

```bash
# From the project root
cmake --build build/Debug
```

If CMake cache is stale after the changes, reconfigure first:
```bash
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
```

---

## Flashing

Use **STM32CubeProgrammer** (GUI or CLI) to flash the generated `build/Debug/Drive-Metrics-AI.elf`:

```bash
# CLI example (adjust serial number if needed)
STM32_Programmer_CLI -c port=SWD -w build/Debug/Drive-Metrics-AI.elf -v -rst
```

---

## Expected UART output

Connect a serial terminal at **115200 baud** to the ST-Link USB (USART1 on PB6/PB7).

On startup:
```
[i] serving_default_keras_tensor_8:0 bytes = 192
[i] serving_default_keras_tensor_8:0 shape = (1, 48)
[i] StatefulPartitionedCall:0 bytes = 24
[i] StatefulPartitionedCall:0 shape = (1, 6)
[*] TFLite model loaded OK
```

Then every 2 seconds:
```
[*] Prediction: [0] Idle State  (xx.x%)
[i]   [0] Idle State             0.xxxx
[i]   [1] Normal Driving         0.xxxx
[i]   [2] Sudden Acceleration    0.xxxx
[i]   [3] Sudden Right Turn      0.xxxx
[i]   [4] Sudden Left Turn       0.xxxx
[i]   [5] Sudden Brake           0.xxxx
```

During bring-up the dummy input (all zeros after normalisation) consistently predicted the same class — expected, since it asks "what does the model predict for the statistical-average input?" With the real sensor pipeline now in place, predictions reflect actual motion.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Linker error: `undefined reference to DebugLog` | `debug_log.cc` is missing from CMakeLists.txt `target_sources` |
| Linker error: `undefined reference to model_tflite` | Symbol rename or `const` was missed in `model_tflite.cc` |
| Linker error: `undefined reference to arm_fully_connected_s8` / `arm_softmax_s8` | Library link order in CMakeLists.txt — `libtflm.a` must come **before** `libcmsis-nn.a`. GNU ld scans archives left-to-right; the library that creates undefined references must precede the one that satisfies them. |
| `AllocateTensors FAILED` on UART | Increase `kTensorArenaSize` (currently 48000) and rebuild |
| No UART output at all | Check baud rate is exactly 115200; check USART1 is selected (PB6/PB7 pins, ST-Link UART) |

## Build memory usage (after all fixes)

Bring-up build (dummy-data, model only):

| Region | Used | Available | % |
|--------|------|-----------|---|
| RAM | 28,344 B | 96 KB | 28.8% |
| FLASH | 113,880 B | 1024 KB | 10.9% |

Current build (full system — IMU + GPS + SD + Wi-Fi/ThingSpeak + collision):

| Region | Used | Available | % |
|--------|------|-----------|---|
| RAM | 63,552 B | 96 KB | 64.7% |
| FLASH | 192,888 B | 1024 KB | 18.4% |

---

## Real sensor data (implemented)

This was the original "next steps" recipe; it is now implemented in
[`main.cpp`](Core/Src/main.cpp) (sampling) and [`classifier.cpp`](Core/Src/classifier.cpp)
(window + features + inference). The dummy feature-vector block in `while(1)` was
replaced by:

1. **LSM6DSL driver** (I2C2, 7-bit address `0x6A` → 8-bit `0xD5` with SA0=GND on this board, via the BSP bus):
   - Write `0x10` to `CTRL1_XL` (accelerometer: 12.5 Hz, ±2 g)
   - Write `0x10` to `CTRL2_G` (gyroscope: 12.5 Hz, ±250 dps)
   - Read 12 bytes from `OUTX_L_G` (0x22) to get 3 gyro + 3 acc int16 values

2. **Collect a 28-sample window** at 2 Hz (one sample every 500 ms) into `float window[28][6]`.

3. **Compute 48 features** — for each of the 6 axes compute: mean, std, min, max, Q1, Q3, RMS, range (exactly as in `window_features()` in the notebook).

4. **Normalise**: `features[i] = (features[i] - SCALER_MEAN[i]) * SCALER_SCALE[i]` for i in 0..47.

5. `memcpy` into `inp->data.f` and call `interpreter.Invoke()` — the rest of the loop is already in place.

Feature order: GyroX (indices 0–7), GyroY (8–15), GyroZ (16–23), AccX (24–31), AccY (32–39), AccZ (40–47). Each group: mean, std, min, max, Q1, Q3, RMS, range.
