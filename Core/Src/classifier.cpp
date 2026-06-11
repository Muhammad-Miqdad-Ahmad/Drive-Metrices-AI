/**
 * Driver-behaviour classifier — TFLite Micro MLP over statistical features
 * of a sliding IMU window (28 samples × 6 channels, 50 % overlap).
 */
#include "classifier.h"
#include "app_log.h"

#include <math.h>
#include <string.h>

#include <model_tflite.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>

#define N_FEATURES 48
#define WINDOW_SIZE 28 /* samples per inference window */
#define STEP_SIZE 14   /* 50 % overlap — run inference every 14 new samples */
#define N_CHANNELS 6   /* GyroX, GyroY, GyroZ, AccX, AccY, AccZ */
#define N_STATS 8      /* mean, std, min, max, Q1, Q3, RMS, range */

constexpr int kTensorArenaSize = 48000;
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

const char *CLASS_NAMES[CLASSIFIER_N_CLASSES] = {
    "Idle State",          "Normal Driving", "Sudden Acceleration",
    "Sudden Right Turn",   "Sudden Left Turn", "Sudden Brake"};

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

static tflite::MicroInterpreter *interpreter;
static TfLiteTensor *inp;
static TfLiteTensor *out;

/* Sliding-window state */
static float win_buf[WINDOW_SIZE][N_CHANNELS];
static int win_head = 0;
static int win_count = 0;
static int steps_since_infer =
    STEP_SIZE; /* ≥ STEP_SIZE → first full window fires immediately */

/* Insertion sort — fast enough for n=28 */
static void isort(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float k = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > k) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = k;
  }
}

/* Compute 8 statistics for one channel into out[0..7]:
   mean, std, min, max, Q1, Q3, RMS, range
   Percentiles use linear interpolation — matches numpy.percentile default. */
static void channel_stats(const float *col, int n, float *stats) {
  float mn = col[0], mx = col[0], sum = 0, sum_sq = 0;
  for (int i = 0; i < n; i++) {
    sum += col[i];
    sum_sq += col[i] * col[i];
    if (col[i] < mn)
      mn = col[i];
    if (col[i] > mx)
      mx = col[i];
  }
  float mean = sum / n, var = 0;
  for (int i = 0; i < n; i++) {
    float d = col[i] - mean;
    var += d * d;
  }

  float s[WINDOW_SIZE];
  memcpy(s, col, (size_t)n * sizeof(float));
  isort(s, n);

  float q1i = 0.25f * (n - 1), q3i = 0.75f * (n - 1);
  int q1l = (int)q1i, q3l = (int)q3i;
  float q1 = s[q1l] + (q1i - q1l) * (s[q1l + 1] - s[q1l]);
  float q3 = s[q3l] + (q3i - q3l) * (s[q3l + 1] - s[q3l]);

  stats[0] = mean;
  stats[1] = sqrtf(var / n);
  stats[2] = mn;
  stats[3] = mx;
  stats[4] = q1;
  stats[5] = q3;
  stats[6] = sqrtf(sum_sq / n);
  stats[7] = mx - mn;
}

int Classifier_Init(void) {
  tflite::InitializeTarget();
  const tflite::Model *model = tflite::GetModel(model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION)
    return -1;

  static tflite::MicroMutableOpResolver<3> resolver;
  resolver.AddFullyConnected();
  resolver.AddRelu();
  resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(model, resolver,
                                                     tensor_arena,
                                                     kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk)
    return -2;

  inp = interpreter->input(0);
  out = interpreter->output(0);
  SUCCESS_PRINTF("TFLite model loaded — input %u bytes, output %u bytes\n",
                 (unsigned)inp->bytes, (unsigned)out->bytes);
  return 0;
}

int Classifier_Push(const float sample[6], Classifier_Result_t *res) {
  for (int ch = 0; ch < N_CHANNELS; ch++)
    win_buf[win_head][ch] = sample[ch];

  win_head = (win_head + 1) % WINDOW_SIZE;
  if (win_count < WINDOW_SIZE)
    win_count++;
  steps_since_infer++;

  if (win_count < WINDOW_SIZE || steps_since_infer < STEP_SIZE)
    return 0;
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

  if (interpreter->Invoke() != kTfLiteOk) {
    DEBUG_PRINTF("Invoke() failed!\n");
    return 0;
  }

  res->best = 0;
  for (int c = 0; c < CLASSIFIER_N_CLASSES; c++) {
    res->probs[c] = out->data.f[c];
    if (res->probs[c] > res->probs[res->best])
      res->best = c;
  }
  return 1;
}
