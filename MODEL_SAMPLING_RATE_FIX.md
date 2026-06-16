# Model Sampling-Rate Fix

How the STM32 firmware feeds IMU data to the TFLite model, what was wrong, and
why the sampling loop was changed to a 2 Hz timer poll.

> Scope: this note is only about **the model and the STM32 code**. It does not
> discuss the data-collection rig or the PC backend.

---

## TL;DR

The model only makes sense if it is fed IMU samples at **2 Hz**. The firmware was
feeding it at roughly **10–12.5 Hz**, so every feature the model sees was computed
over the wrong slice of time and the predictions were unreliable. The sampling
loop in `Core/Src/main.cpp` was rewritten to poll the sensor every **500 ms**
(2 Hz), which restores the rate the model was trained on.

---

## 1. How the model consumes IMU data (the contract that matters)

The on-device classifier (`Core/Src/classifier.cpp`) does this for every inference:

1. Collect a **sliding window of 28 samples** of `[GyroX, GyroY, GyroZ, AccX, AccY, AccZ]`,
   with 50 % overlap (a new window every 14 samples).
2. For each of the 6 channels, compute **8 statistics** — mean, std, min, max,
   Q1, Q3, RMS, range — giving a **48-element feature vector**.
3. Normalise that vector with the `StandardScaler` constants baked into the file.
4. Run it through the MLP (`48 → 64 → 32 → 6`, softmax).

The hidden assumption in step 1 is **the sample rate**. The model was trained with
windows captured at **2 Hz**, so:

| Quantity | At 2 Hz (what the model expects) |
|---|---|
| 28-sample window covers | 28 × 0.5 s = **14 seconds** of motion |
| New prediction every | 14 samples = **7 seconds** |

The 48 features are therefore summaries of **14 seconds** of driving. The model
learned the distribution of *14-second* statistics. **Sample rate is the contract
between the raw data and the model** — if the rate is wrong, the window covers a
different amount of time, the statistics describe something the model never saw,
and the scaled inputs fall outside the range the network was trained on.

---

## 2. The problem

The firmware was **not** delivering samples at 2 Hz. Two independent issues stacked:

1. **The sampling loop free-ran at ~10 Hz.** The loop was meant to advance once per
   data-ready interrupt, but that path hung (see §3), so it had been patched to
   self-trigger and was paced only by a `HAL_Delay(100)` → about **10 reads/second**.

2. **The sensor itself was running at 12.5 Hz, not 2 Hz.** `MEMS_Init` calls
   `LSM6DSL_ACC_SetOutputDataRate(MotionSensor, 2.0f)`, but the LSM6DSL has **no 2 Hz
   rate**. The driver clamps anything ≤ 12.5 to its lowest normal rate:

   ```c
   // Drivers/.../lsm6dsl.c
   new_odr = (Odr <= 12.5f) ? LSM6DSL_XL_ODR_12Hz5 : ... ;
   ```

**Effect on the model:** fed at ~10–12.5 Hz, a 28-sample window covered only
**~2.2–2.8 seconds** instead of 14, and inference fired **~5–6× too often**. Every
mean/std/min/max/Q1/Q3/RMS/range was computed over the wrong time scale, so the
normalised feature vector sat outside the model's training distribution and the
softmax outputs were not trustworthy.

---

## 3. What the code did before

The main loop was gated on a data-ready interrupt flag, with a fallback that
defeated the gate:

```c
if (dataRdyIntReceived != 0) {
    dataRdyIntReceived = 0;
    /* read sensor + Classifier_Push + log */
    HAL_Delay(100);
} else {
    dataRdyIntReceived = 1;   // self-trigger: makes the loop free-run
}
```

When the loop was gated **purely** on the interrupt (`if` only, no `else`), it hung
and never advanced. The `else` branch was added as a workaround: it sets the flag
itself, so the loop runs every iteration, paced only by `HAL_Delay(100)` (~10 Hz).

**Why the pure-interrupt version hung:** the accelerometer's data-ready (DRDY) line
was left in **latched** mode — it goes high when a sample is ready and stays high
until the data registers are read — while the STM32 EXTI on that pin is
**rising-edge**. A rising-edge input cannot re-trigger on a line that is already
high. So once the flag was cleared with the line still latched high (which happens
right after the first sample, during start-up init), no further edge ever arrived,
the ISR never ran again, and the loop blocked forever on the `if`.

So the firmware was stuck choosing between *"hangs"* and *"runs at the wrong rate"* —
and neither delivered the 2 Hz the model needs.

---

## 4. Root causes, summarised

| # | Root cause | Consequence |
|---|---|---|
| 1 | Loop free-ran at ~10 Hz (the `else` workaround) | Wrong feed rate to the model |
| 2 | LSM6DSL has no 2 Hz ODR; `2.0f` → 12.5 Hz | Sensor streamed at 12.5 Hz |
| 3 | Latched DRDY + rising-edge EXTI | Pure-interrupt loop hung (reason the `else` existed) |

---

## 5. Options considered

**Option A — repair the interrupt path.**
Switch the sensor to DRDY *pulse* mode (`LSM6DSL_Set_DRDY_Mode(MotionSensor, 1)`) so
every sample emits a clean edge (fixes the hang), then **decimate** the 12.5 Hz
interrupt stream to ~2 Hz in software (act on every 6th interrupt).

- ✔ Event-driven; can sleep between samples.
- ✗ More moving parts (pulse-mode config + decimation counter + ISR/main shared
  state). Its only real benefit — low-power sleep — is marginal here, because the
  sensor still wakes the CPU 12.5×/s and 5 of every 6 wakes are discarded.

**Option B — poll on a timer (chosen).**
Run the sensor at its real rate and read the **latest** sample every **500 ms**
using `HAL_GetTick()`, which decimates the 12.5 Hz stream down to the 2 Hz the
model wants.

- ✔ Directly and reliably produces 2 Hz.
- ✔ Fewest moving parts; no edge/latch fragility, so the hang cannot recur.
- ✔ The data-ready interrupt is not on the critical path at all.

---

## 6. Decision — and why

**Chose Option B (timer poll).** For a 2 Hz application it produces exactly the rate
the model's window contract requires, with the least code and no dependence on the
fragile latched-DRDY/edge behaviour that caused the original hang. The one thing the
interrupt buys you — sleeping between samples — is negligible when the sensor's
lowest rate (12.5 Hz) already forces frequent wake-ups. Simpler, correct, and robust
won.

---

## 7. What changed

**Only the `while (1)` loop in `Core/Src/main.cpp`.** The interrupt-flag gate, its
`else` self-trigger, and the trailing `HAL_Delay(100)` were replaced with a
`HAL_GetTick()` 500 ms gate plus a catch-up clamp:

```c
static uint32_t lastSampleMs = 0;
uint32_t now = HAL_GetTick();
if (now - lastSampleMs >= 500) {          // 2 Hz
    lastSampleMs += 500;
    if (now - lastSampleMs > 500)         // fell >1 period behind (slow I/O) → resync
        lastSampleMs = now;
    /* read sensor + Classifier_Push + log  — unchanged */
}
```

Everything else is untouched:

- The sensor read, the mg→g / mdps→dps unit conversion, the `Classifier_Push`
  call, the prediction print-out, and the GPS / SD-log code are **byte-for-byte the
  same**.

> **Since superseded:** the cloud path has since moved from Supabase to **ThingSpeak**
> (geofenced upload, [`thingspeak.c`](Core/Src/thingspeak.c)), and the INT1 line that
> originally carried the data-ready interrupt has been **repurposed for the LSM6DSL
> hardware collision wake-up** (see the main [`README.md`](README.md) §10). The 2 Hz
> timer-poll described here is unchanged — collision detection runs independently in
> the sensor and does not touch the sampling loop.

---

## 8. Why it's better (back to the model)

With reads spaced exactly 500 ms apart:

- a 28-sample window again represents **14 seconds**, and inference fires every
  **7 seconds** — the same cadence the model was trained on;
- the 48 statistics summarise the **same time span** the model learned, so the
  normalised feature vector lands back inside the training distribution and the
  softmax outputs become meaningful;
- the **catch-up clamp** prevents a slow logging/SD iteration from firing a burst of
  back-to-back reads. Without it, those reads (faster than the sensor's 80 ms update)
  would return near-identical values, shrinking the window's std/range and distorting
  the features all over again.

---

## 9. What this does NOT fix

Correct rate is necessary but **not sufficient** for trustworthy predictions. The
model was trained on a dataset whose **GyroY channel carries a roughly constant
+4 °/s offset** (a zero-rate bias of the sensor that recorded the data). This board's
LSM6DSL reads GyroY near **0**, so after the `StandardScaler` step that one feature
lands several standard deviations away from where the model expects it — enough on its
own to skew the result. There are also likely axis-orientation / sign differences.

Fixing this needs **retraining the model on data captured with this board's sensor**
(or applying a per-axis calibration before `Classifier_Push`). It is a separate,
still-open item and is unaffected by the sampling-rate change.

---

## 10. Notes

- `MEMS_Init` still requests `2.0f` (resolves to **12.5 Hz**, the LSM6DSL's lowest
  rate), then enabling the collision wake-up bumps the **accelerometer ODR to 416 Hz**
  internally. Either way the poll loop reads the *latest* sample every 500 ms, so the
  model still sees 2 Hz — the hardware ODR only sets how fresh that latest sample is.
- The wait is a busy-spin (no sleep). Functionally fine; add `__WFI()` in the wait if
  power ever matters.
- The feature-extraction math, the scaler constants, and the model I/O in
  `classifier.cpp` were verified correct and were **not** touched. The only defect on
  the path from sensor to model was the sample rate.
