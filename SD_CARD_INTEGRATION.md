# SD Card Module Integration — B-L475E-IOT01A1

## Hardware

**Module:** Generic SPI SD card module with AMS1117 (3.3V regulator) and LXQ1AT520A level shifter.

### Wiring

| SD Module Pin | STM32 Pin | Arduino Label | Notes |
|---------------|-----------|---------------|-------|
| VCC           | 5V        | 5V            | Module has onboard 3.3V regulator — needs 5V input |
| GND           | GND       | GND           | — |
| SCK           | PA5       | D13           | SPI1_SCK |
| MISO          | PA6       | D12           | SPI1_MISO |
| MOSI          | PA7       | D11           | SPI1_MOSI |
| CS            | PA2       | D10           | GPIO output, active low |

### CubeMX Settings

- **SPI1**: Mode = Full-Duplex Master, NSS = Software, Data Size = 8-bit, CPOL = Low, CPHA = 1 Edge
- **PA2**: GPIO Output (manual chip-select, not SPI NSS)
- **FatFS middleware**: enabled via CubeMX

---

## Problems and Fixes

### 1. CubeMX wiped the diskio driver

**Problem:** Every time CubeMX regenerated code, it replaced `FATFS/Target/user_diskio.c` with empty stubs. The entire SPI SD card driver (CMD0/8/41/58, read/write sectors) was lost.

**Fix:** The driver must be rewritten after every CubeMX regeneration. All custom code lives inside the `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks which CubeMX should preserve — but the full function bodies were outside those guards.

---

### 2. SPI clock too fast for SD initialisation

**Problem:** SPI1 was configured with `SPI_BAUDRATEPRESCALER_2`, giving 80 MHz ÷ 2 = **40 MHz**. The SD card SPI specification requires ≤ 400 kHz during the initialisation sequence (CMD0, CMD8, ACMD41). The card received commands it could not process and returned error responses.

**Fix:** Changed to `SPI_BAUDRATEPRESCALER_256`, giving 80 MHz ÷ 256 = **312 kHz** — safely within the init window.

```c
hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
```

---

### 3. CS pin driven LOW at boot

**Problem:** The GPIO init called:
```c
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | ..., GPIO_PIN_RESET);
```
This pulled CS **low** (asserted) from the moment the MCU powered on. Any SPI activity before `USER_initialize()` was seen by the card with CS active, putting it into a confused state. CMD0 returned `0x3F` (multiple error bits) instead of `0x01`.

**Fix:** Initialise PA2 **high** (deasserted) at boot:
```c
HAL_GPIO_WritePin(GPIOA, SPBTLE_RF_RST_Pin | ARD_D9_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); /* SD CS — deasserted */
```

---

### 4. SD card formatted as exFAT

**Problem:** The SD card previously held a Raspberry Pi image (boot + rootfs partitions). After erasing it on a Xiaomi Android phone, the filesystem was not recognised by FatFS (`f_open` returned `FR_INVALID_NAME`).

**Fix:** Added `f_mkfs` auto-format in `SD_Log_Init()`. On first boot with an unrecognised filesystem, the STM32 formats the card as FAT32 itself:

```c
FRESULT res = f_mount(&USERFatFS, USERPath, 1);
if (res != FR_OK) {
    BYTE work[512];
    f_mkfs(USERPath, FM_FAT32, 0, work, sizeof(work));
    f_mount(&USERFatFS, USERPath, 1);
}
```

---

### 5. Long filename rejected — `log.json` invalid

**Problem:** FatFS was configured with `_USE_LFN = 0` (long filename support disabled). The filename `log.json` has a 4-character extension (`json`), which exceeds the FAT 8.3 limit of 3 characters. `f_open` returned `FR_INVALID_NAME` (error code 6).

**Fix:** Renamed the log file to `data.log` — a valid 8.3 name (4-char base, 3-char extension). The file content is JSON Lines.

```c
f_open(&log_file, "0:/data.log", FA_CREATE_ALWAYS | FA_WRITE);
```

---

### 6. ThingSpeak rejected the upload — `error_duplicate_timestamps`

**Problem:** Originally the log was opened with `FA_OPEN_ALWAYS` + seek-to-end (append), so it accumulated records across many boots. The `time` field is `HAL_GetTick()` (ms since *this* boot, restarting at 0 each power-up). At upload time every session's ticks were converted to wall-clock using the *current* GPS anchor, so records from different runs collapsed onto identical `created_at` seconds — and ThingSpeak rejects a bulk update with duplicate timestamps (HTTP 400).

**Fix (two layers):**
- **Truncate on boot** — `SD_Log_Init()` now opens with `FA_CREATE_ALWAYS`, so one file only ever holds a single power session (monotonic ticks).
- **Strictly-increasing guard** in `thingspeak.c` — each record's epoch second is bumped to `prev + 1` if it isn't strictly greater than the previous one, so even a tie can't reach the API.

(Trade-off: a mid-trip reset loses that session's data. Drop the clear-on-boot if cross-reboot persistence matters more than keeping one session per file — the uploader guard alone still prevents the 400.)

---

## Log Format

Each prediction writes one JSON line to `0:/data.log`:

```json
{"time":512000,"pred":1,"label":"Normal Driving","conf":92.4,
 "lat":31.515000,"lon":74.465000,"spd":18.3,"fix":1,
 "gmax":1.07,"collision":0,"gpeak":0.00,
 "gx":[...28...],"gy":[...],"gz":[...],"ax":[...],"ay":[...],"az":[...]}
```

| Field        | Description |
|--------------|-------------|
| `time`       | Timestamp in milliseconds since boot (`HAL_GetTick()`) |
| `pred`       | Predicted class index (0–5) |
| `label`      | Predicted class name |
| `conf`       | Confidence percentage (0–100) |
| `lat` / `lon`| GPS position in decimal degrees (N / E positive) |
| `spd`        | Speed in km/h from GPS |
| `fix`        | 1 = active GPS fix, 0 = no fix |
| `gmax`       | Peak \|a\| over the window in g (harshness) |
| `collision`  | 1 on the first record after a detected impact, else 0 |
| `gpeak`      | Peak g read at the impact (0 when `collision` = 0) |
| `gx … az`    | The entire 28-sample raw window the model classified (6 arrays) |

The file is flushed to disk every 10 records, truncated on boot, and cleared after a successful ThingSpeak upload. Remove the card while the board is powered off to avoid data loss.

### Reading on PC

```python
import json

records = [json.loads(line) for line in open("data.log") if line.strip()]
for r in records:
    print(r["time"], r["label"], r["conf"], r["gmax"], r["collision"])
```
