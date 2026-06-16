# Driver Behaviour Detection on STM32L475 — How It Was Built

> **Scope:** this is the original walk-through for getting the **TFLite Micro model
> running on the board** (train → export → embed → first inference on the device).
> The firmware has since grown real IMU sampling, GPS, SD logging, ThingSpeak upload,
> and hardware collision detection — for the full current system see
> [`README.md`](README.md). The TFLM build steps below are still accurate; the
> initial "dummy data" inference loop has been replaced by the real sensor pipeline
> in [`classifier.cpp`](Core/Src/classifier.cpp) (noted inline where relevant).

Target board: **B-L475E-IOT01A** (STM32L475VG, Cortex-M4F, 1 MB flash, 128 KB SRAM)  
Framework: **TensorFlow Lite Micro (TFLM)**  
Model: 6-class MLP — Idle / Normal Driving / Sudden Acceleration / Sudden Right Turn / Sudden Left Turn / Sudden Brake

---

## 1. Python — Train and Export

### 1a. Train the model

A standard Keras MLP trained on normalised IMU features (48 features per sample).  
A `StandardScaler` is fitted on the training data; its `mean_` and `scale_` arrays are copied into `main.cpp` as `SCALER_MEAN` and `SCALER_SCALE`.

### 1b. Export to TFLite — pure float32, no quantisation

```python
converter = tf.lite.TFLiteConverter.from_keras_model(mlp_model)
tflite_model = converter.convert()          # float32 weights and activations
with open('driver_behaviour_mlp.tflite', 'wb') as f:
    f.write(tflite_model)
```

**Do not add `converter.optimizations`.** This TFLM version only supports
`float32 × float32` or `int8 × int8` in its FullyConnected kernel.  
A mixed (hybrid) model — float32 activations with INT8 weights — reads INT8 weight
bytes as float32 and silently produces NaN output.

---

## 2. Embed the Model in the Firmware

Run once from the project root whenever `driver_behaviour_mlp.tflite` changes:

```bash
echo '#include "model_tflite.h"' > Core/Src/model_tflite.cc && \
xxd -i driver_behaviour_mlp.tflite \
  | sed '1s/.*/const unsigned char model_tflite[] = {/' \
  | sed '/^unsigned int/d' \
  >> Core/Src/model_tflite.cc
```

`Core/Inc/model_tflite.h` contains only:

```c
extern const unsigned char model_tflite[];
```

---

## 3. Linker Script — `STM32L475XX_FLASH.ld`

Stack size increased from the CubeMX default (1 KB) to 8 KB to support the TFLM
interpreter + `printf`/`vfprintf` call depth:

```ld
_Min_Stack_Size = 0x2000;   /* 8 KB */
```

---

## 4. CMake — `cmake/gcc-arm-none-eabi.cmake`

Added `-u _printf_float` so newlib-nano's `printf` handles `%f` / `%.4f`:

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs -u _printf_float")
```

---

## 5. Embedding TFLM in the firmware — Key Additions

> The TFLM runtime (arena, scaler, op resolver, interpreter) now lives in
> [`classifier.cpp`](Core/Src/classifier.cpp), not `main.cpp`. During bring-up it was
> all in `main.cpp` with a dummy feature vector; the snippets below show that
> original form.

### 5a. Tensor arena

```cpp
const int kTensorArenaSize = 48000;          // 48 KB — plenty for this model
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
```

### 5b. Scaler constants (copied from notebook output)

```cpp
static const float SCALER_MEAN[48]  = { /* 48 values from scaler.mean_  */ };
static const float SCALER_SCALE[48] = { /* 48 values from scaler.scale_ */ };
static const char *CLASS_NAMES[6]   = {
    "Idle State", "Normal Driving", "Sudden Acceleration",
    "Sudden Right Turn", "Sudden Left Turn", "Sudden Brake"
};
#define N_FEATURES 48
```

### 5c. `_write` syscall — UART output with CR+LF

```cpp
extern "C" int _write(int fd, char *ptr, int len) {
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n') {
            uint8_t cr = '\r';
            HAL_UART_Transmit(&huart1, &cr, 1, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)&ptr[i], 1, HAL_MAX_DELAY);
    }
    return len;
}
```

`(void)fd` ensures both stdout (fd=1) and stderr (fd=2) reach UART — TFLM logs
error messages on stderr, so without this they are silently dropped.

### 5d. TFLM initialisation (USER CODE BEGIN 2)

```cpp
tflite::InitializeTarget();
const tflite::Model *tfl_model = tflite::GetModel(model_tflite);
assert(tfl_model->version() == TFLITE_SCHEMA_VERSION);

static tflite::MicroMutableOpResolver<3> resolver;
resolver.AddFullyConnected();
resolver.AddRelu();
resolver.AddSoftmax();

static tflite::MicroInterpreter interpreter(
    tfl_model, resolver, tensor_arena, kTensorArenaSize);

if (interpreter.AllocateTensors() != kTfLiteOk) {
    DEBUG_PRINTF("AllocateTensors FAILED\n");
    Error_Handler();
}

TfLiteTensor *inp = interpreter.input(0);
TfLiteTensor *out = interpreter.output(0);
```

### 5e. Inference loop (while 1) — bring-up placeholder

This proved the pipeline with a dummy all-zero input. **It has since been replaced**
by real IMU sampling + feature extraction in
[`classifier.cpp`](Core/Src/classifier.cpp) (28×6 sliding window, 48 statistical
features, gyro mean-centring); see [`MODEL_SAMPLING_RATE_FIX.md`](MODEL_SAMPLING_RATE_FIX.md).
The original placeholder block:

```cpp
// Replace this block with real LSM6DSL sensor data + feature extraction.
// Normalisation formula: norm = (raw - mean) / scale
float features[N_FEATURES];
for (int i = 0; i < N_FEATURES; i++) {
    features[i] = (0.0f - SCALER_MEAN[i]) / SCALER_SCALE[i]; // placeholder: raw=0
}

memcpy(inp->data.f, features, N_FEATURES * sizeof(float));

if (interpreter.Invoke() != kTfLiteOk) {
    DEBUG_PRINTF("Invoke() failed!\n");
} else {
    int best = 0;
    float prob = out->data.f[0];
    for (int c = 1; c < 6; c++) {
        if (out->data.f[c] > prob) { prob = out->data.f[c]; best = c; }
    }
    SUCCESS_PRINTF("Prediction: [%d] %s  (%.1f%%)\n",
                   best, CLASS_NAMES[best], prob * 100.0f);
    for (int c = 0; c < 6; c++) {
        DEBUG_PRINTF("  [%d] %-24s %.4f\n", c, CLASS_NAMES[c], out->data.f[c]);
    }
}
HAL_Delay(2000);
```

---

## 6. Serial Monitor

USART1 (ST-Link virtual COM) runs at **115200 baud**. Connect with:

```bash
picocom -b 115200 /dev/ttyACM0
```

---

## 7. Real Sensor Data (done)

The placeholder is gone — [`classifier.cpp`](Core/Src/classifier.cpp) now feeds the
model real IMU windows. `main.cpp` reads the LSM6DSL every 500 ms (2 Hz) and pushes
`[GyroX,GyroY,GyroZ,AccX,AccY,AccZ]` into a 28-sample sliding window; on each
completed window the classifier extracts 48 statistical features, mean-centres the
gyro axes, normalises with the baked-in `StandardScaler` constants, and runs
inference. The scaler constants match the Python training pipeline.

See [`README.md`](README.md) for how the prediction then flows to GPS tagging, SD
logging, ThingSpeak upload, and collision detection.
