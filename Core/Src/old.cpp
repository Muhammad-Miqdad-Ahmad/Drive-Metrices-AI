/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <model_tflite.h>
#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_log.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/micro/micro_profiler.h>
#include <tensorflow/lite/micro/micro_op_resolver.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/recording_micro_interpreter.h>

#include "lsm6dsl.h"
#include "b_l475e_iot01a1_bus.h"

#include <dsp/basic_math_functions.h>
#include <dsp/fast_math_functions.h>
#include <dsp/transform_functions.h>
#include <dsp/window_functions.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const int kTensorArenaSize = 48000;
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

/* USER CODE END PD */

#define DEBUG_PRINTF(...)                                                      \
  {                                                                            \
    printf("[i] ");                                                            \
    printf(__VA_ARGS__);                                                       \
  }

#define RAW_PRINTF(...)                                                        \
  {                                                                            \
    printf(__VA_ARGS__);                                                       \
  }

#define SUCCESS_PRINTF(...)                                                    \
  {                                                                            \
    printf("[*] ");                                                            \
    printf(__VA_ARGS__);                                                       \
  }

/* Scaler constants from notebook — do not modify */
static const float SCALER_MEAN[48] = {
    -0.52660075f, 0.92552432f,  -2.46971791f, 1.42646365f,  -1.08713346f,
    0.02858994f,  1.27643845f,  3.89618156f,  4.16019767f,  1.45395997f,
    0.95809761f,  7.31645806f,  3.32783283f,  4.99534288f,  4.73456197f,
    6.35836046f,  0.50716550f,  1.28830385f,  -2.02102969f, 2.65010864f,
    -0.24544190f, 1.32955882f,  3.53321934f,  4.67113833f,  0.15297517f,
    0.04737416f,  0.07190084f,  0.24876506f,  0.11952566f,  0.18406218f,
    0.21576459f,  0.17686422f,  -0.08225364f, 0.03524866f,  -0.15398596f,
    -0.01671209f, -0.10452421f, -0.05852724f, 0.10989266f,  0.13727387f,
    -0.44440470f, 0.04404843f,  -0.53592140f, -0.35092079f, -0.47071570f,
    -0.41798094f, 1.00030717f,  0.18500062f};
static const float SCALER_SCALE[48] = {
    0.79834809f,  1.11725889f, 2.70796325f, 2.99170242f, 0.96787476f,
    1.03103484f,  1.18059936f, 5.23526673f, 0.81953465f, 2.23786787f,
    5.36808023f,  5.37689118f, 1.48760115f, 1.48432866f, 1.64184263f,
    10.34018406f, 4.68461919f, 1.98718123f, 6.58826100f, 6.07888270f,
    4.84926212f,  4.72339753f, 3.91587024f, 6.47268623f, 0.16261260f,
    0.04741832f,  0.20110329f, 0.14970457f, 0.17302503f, 0.16472156f,
    0.08845170f,  0.17825900f, 0.08004790f, 0.03592624f, 0.11058524f,
    0.11497202f,  0.08198896f, 0.08454913f, 0.06047130f, 0.14238991f,
    0.89173137f,  0.06775713f, 0.85308819f, 0.95136172f, 0.88208573f,
    0.90391908f,  0.03521959f, 0.26578198f};
static const char *CLASS_NAMES[6] = {"Idle State",          "Normal Driving",
                                     "Sudden Acceleration", "Sudden Right Turn",
                                     "Sudden Left Turn",    "Sudden Brake"};
#define N_FEATURES  48
#define WINDOW_SIZE 28   /* samples per inference window */
#define STEP_SIZE   14   /* 50 % overlap — run inference every 14 new samples */
#define N_CHANNELS   6   /* GyroX, GyroY, GyroZ, AccX, AccY, AccZ */
#define N_STATS      8   /* mean, std, min, max, Q1, Q3, RMS, range */

void PrintModelDetails(const tflite::Model *model) {
  DEBUG_PRINTF("TFlite schema version: %lu\n", model->version());
  assert(model->version() == TFLITE_SCHEMA_VERSION);

  // Primary subgraph usually at index 0
  const tflite::SubGraph *subgraph = model->subgraphs()->Get(0);

  DEBUG_PRINTF("Number of tensors: %lu\n", subgraph->tensors()->size());
  for (size_t i = 0; i < subgraph->tensors()->size(); i++) {
    const tflite::Tensor *tensor = subgraph->tensors()->Get(i);
    DEBUG_PRINTF("    %u: %s\n", i,
                 (char *)tflite::EnumNameTensorType(tensor->type()));
  }
  DEBUG_PRINTF("Number of operators: %lu\n", subgraph->operators()->size());

  // Iterate over operators and print their details
  for (size_t i = 0; i < subgraph->operators()->size(); i++) {
    const tflite::Operator *op = subgraph->operators()->Get(i);
    const tflite::OperatorCode *op_code =
        model->operator_codes()->Get(op->opcode_index());

    const char *op_name = tflite::EnumNameBuiltinOperator(
        static_cast<tflite::BuiltinOperator>(op_code->builtin_code()));

    DEBUG_PRINTF("    %u: %s\n", i, op_name);

    DEBUG_PRINTF("        Inputs: ");
    for (size_t j = 0; j < op->inputs()->size(); j++) {
      printf("%lu ", op->inputs()->Get(j));
    }
    printf("\n");

    DEBUG_PRINTF("        Outputs: ");
    for (size_t j = 0; j < op->outputs()->size(); j++) {
      printf("%lu ", op->outputs()->Get(j));
    }
    printf("\n");
  }
}

/* Insertion sort — fast enough for n=28 */
static void isort(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float k = a[i]; int j = i - 1;
    while (j >= 0 && a[j] > k) { a[j+1] = a[j]; j--; }
    a[j+1] = k;
  }
}

/* Compute 8 statistics for one channel into out[0..7]:
   mean, std, min, max, Q1, Q3, RMS, range
   Percentiles use linear interpolation — matches numpy.percentile default. */
static void channel_stats(const float *col, int n, float *out) {
  float mn = col[0], mx = col[0], sum = 0, sum_sq = 0;
  for (int i = 0; i < n; i++) {
    sum    += col[i];  sum_sq += col[i] * col[i];
    if (col[i] < mn) mn = col[i];
    if (col[i] > mx) mx = col[i];
  }
  float mean = sum / n, var = 0;
  for (int i = 0; i < n; i++) { float d = col[i] - mean; var += d * d; }

  float s[WINDOW_SIZE];
  memcpy(s, col, (size_t)n * sizeof(float));
  isort(s, n);

  float q1i = 0.25f * (n - 1), q3i = 0.75f * (n - 1);
  int   q1l = (int)q1i,         q3l = (int)q3i;
  float q1  = s[q1l] + (q1i - q1l) * (s[q1l + 1] - s[q1l]);
  float q3  = s[q3l] + (q3i - q3l) * (s[q3l + 1] - s[q3l]);

  out[0] = mean;           out[1] = sqrtf(var / n);
  out[2] = mn;             out[3] = mx;
  out[4] = q1;             out[5] = q3;
  out[6] = sqrtf(sum_sq / n);  out[7] = mx - mn;
}

void print_shape(TfLiteTensor *tensor) {
  const char *name = (tensor->name != nullptr) ? tensor->name : "(unnamed)";
  DEBUG_PRINTF("%s bytes = %u\n", name, (unsigned)tensor->bytes);
  DEBUG_PRINTF("%s shape = (", name);
  for (int i = 0; i < tensor->dims->size; i++) {
    RAW_PRINTF("%d", tensor->dims->data[i]);
    if (i != tensor->dims->size - 1) {
      RAW_PRINTF(", ");
    }
  }
  RAW_PRINTF(")\n");
}
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
LSM6DSL_Object_t MotionSensor;
volatile uint32_t dataRdyIntReceived;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  dataRdyIntReceived = 0;
  // For the inbuilt accelerometer
  MEMS_Init();

  tflite::InitializeTarget();
  const tflite::Model *tfl_model = tflite::GetModel(model_tflite);
  assert(tfl_model->version() == TFLITE_SCHEMA_VERSION);

  static tflite::MicroMutableOpResolver<3> resolver;
  resolver.AddFullyConnected();
  resolver.AddRelu();
  resolver.AddSoftmax();

  static tflite::MicroInterpreter interpreter(tfl_model, resolver, tensor_arena,
                                              kTensorArenaSize);

  if (interpreter.AllocateTensors() != kTfLiteOk) {
    DEBUG_PRINTF("AllocateTensors FAILED\n");
    Error_Handler();
  }

  TfLiteTensor *inp = interpreter.input(0);
  TfLiteTensor *out = interpreter.output(0);
  SUCCESS_PRINTF("TFLite model loaded — input %u bytes, output %u bytes\n",
                 (unsigned)inp->bytes, (unsigned)out->bytes);

  /* Sliding-window state (28 samples × 6 channels, 50 % overlap) */
  static float win_buf[WINDOW_SIZE][N_CHANNELS] = {};
  static int   win_head          = 0;
  static int   win_count         = 0;
  static int   steps_since_infer = STEP_SIZE; /* ≥ STEP_SIZE → first full window fires immediately */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (dataRdyIntReceived != 0) {
      dataRdyIntReceived = 0;

      LSM6DSL_Axes_t acc, gyro;
      LSM6DSL_ACC_GetAxes(&MotionSensor, &acc);
      LSM6DSL_GYRO_GetAxes(&MotionSensor, &gyro);

      /* Channel order: [GyroX, GyroY, GyroZ, AccX, AccY, AccZ]
         Units: mdps ÷ 1000 → dps,   mg ÷ 1000 → g   (match CSV training data) */
      win_buf[win_head][0] = (float)gyro.x / 1000.0f;
      win_buf[win_head][1] = (float)gyro.y / 1000.0f;
      win_buf[win_head][2] = (float)gyro.z / 1000.0f;
      win_buf[win_head][3] = (float)acc.x  / 1000.0f;
      win_buf[win_head][4] = (float)acc.y  / 1000.0f;
      win_buf[win_head][5] = (float)acc.z  / 1000.0f;

      win_head = (win_head + 1) % WINDOW_SIZE;
      if (win_count < WINDOW_SIZE) win_count++;
      steps_since_infer++;

      if (win_count == WINDOW_SIZE && steps_since_infer >= STEP_SIZE) {
        steps_since_infer = 0;

        /* Extract 48 statistical features (8 stats × 6 channels) */
        float col[WINDOW_SIZE], features[N_FEATURES];
        for (int ch = 0; ch < N_CHANNELS; ch++) {
          for (int t = 0; t < WINDOW_SIZE; t++)
            col[t] = win_buf[(win_head + t) % WINDOW_SIZE][ch]; /* oldest first */
          channel_stats(col, WINDOW_SIZE, features + ch * N_STATS);
        }

        /* Normalise with StandardScaler and copy into model input */
        for (int i = 0; i < N_FEATURES; i++)
          inp->data.f[i] = (features[i] - SCALER_MEAN[i]) / SCALER_SCALE[i];

        if (interpreter.Invoke() != kTfLiteOk) {
          DEBUG_PRINTF("Invoke() failed!\n");
        } else {
          int best = 0; float prob = out->data.f[0];
          for (int c = 1; c < 6; c++)
            if (out->data.f[c] > prob) { prob = out->data.f[c]; best = c; }
          SUCCESS_PRINTF("Prediction: [%d] %s  (%.1f%%)\n",
                         best, CLASS_NAMES[best], prob * 100.0f);
          for (int c = 0; c < 6; c++)
            DEBUG_PRINTF("  [%d] %-24s %.4f\n", c, CLASS_NAMES[c], out->data.f[c]);
        }
      }
      HAL_Delay(100);
    }
    else
    {
      // DEBUG_PRINTF("No data ready interrupt received yet.\n");
      dataRdyIntReceived = 1;
    }
    /* USER CODE END 3 */
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
   */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void) {

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection =
      DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock =
      DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void) {

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOE, M24SR64_Y_RF_DISABLE_Pin | M24SR64_Y_GPO_Pin | ISM43362_RST_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin | SPBTLE_RF_RST_Pin | ARD_D9_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin |
                        LED2_Pin | SPSGRF_915_SDN_Pin | ARD_D5_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOD, USB_OTG_FS_PWR_EN_Pin | PMOD_RESET_Pin | STSAFE_A100_RESET_Pin,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin
   * ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin | M24SR64_Y_GPO_Pin |
                        ISM43362_RST_Pin | ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin
   * SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin |
                        SPSGRF_915_GPIO3_EXTI5_Pin | SPBTLE_RF_IRQ_EXTI6_Pin |
                        ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin | ARD_A4_Pin | ARD_A3_Pin | ARD_A2_Pin |
                        ARD_A1_Pin | ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin | ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin | SPBTLE_RF_RST_Pin | ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin | ARD_D12_Pin | ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin
     LED2_Pin SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin |
                        LED2_Pin | SPSGRF_915_SDN_Pin | ARD_D5_Pin |
                        SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin PD11 ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin | GPIO_PIN_11 |
                        ARD_D2_Pin | HTS221_DRDY_EXTI15_Pin |
                        PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin
   * PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin | SPBTLE_RF_SPI3_CSN_Pin |
                        PMOD_RESET_Pin | STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin | LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin
   * PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin | PMOD_UART2_RTS_Pin |
                        PMOD_UART2_TX_Pin | PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin | ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
  /* EXTI9_5_IRQn — covers PD11 (LSM6DSL INT1 on EXTI line 11) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
static void MEMS_Init(void) {
  LSM6DSL_IO_t io_ctx;

  /* Wire the sensor to the BSP I2C2 bus (onboard sensor bus on B-L475E-IOT01A1) */
  io_ctx.BusType  = 0;                   /* 0 = I2C */
  io_ctx.Address  = LSM6DSL_I2C_ADD_L;  /* SA0=GND on this board → 0xD5 */
  io_ctx.Init     = BSP_I2C2_Init;
  io_ctx.DeInit   = BSP_I2C2_DeInit;
  io_ctx.ReadReg  = BSP_I2C2_ReadReg;
  io_ctx.WriteReg = BSP_I2C2_WriteReg;
  io_ctx.GetTick  = BSP_GetTick;
  io_ctx.Delay    = HAL_Delay;

  LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);
  LSM6DSL_Init(&MotionSensor);

  /* Enable both sensors at 2 Hz — matches the training data ODR */
  LSM6DSL_ACC_Enable(&MotionSensor);
  LSM6DSL_GYRO_Enable(&MotionSensor);
  LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 2.0f);
  LSM6DSL_GYRO_SetOutputDataRate(&MotionSensor, 2.0f);

  /* Route accelerometer data-ready to INT1 (PD11) */
  LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_11) {   /* LSM6DSL INT1 on PD11 */
    dataRdyIntReceived = 1;
  }
}

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
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  DEBUG_PRINTF("*** Error_Handler reached — halting ***\n");
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
    HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
