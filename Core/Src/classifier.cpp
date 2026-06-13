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
/* Gyro-mean features (indices 0, 8, 16) are 0 with zero variance after
   per-window centering; their SCALER_SCALE is 0 and is guarded below. */
static const float SCALER_MEAN[48] = {
    -0.00000000f, 0.92646079f,  -1.94429062f, 1.95204022f,  -0.56155014f,
    0.55802498f,  0.92646079f,  3.89633084f,  -0.00000000f, 1.45713168f,
    -3.20900120f, 3.15741605f,  -0.83468869f, 0.83774063f,  1.45713168f,
    6.36641725f,  0.00000000f,  1.29126623f,  -2.52620138f, 2.15306731f,
    -0.75612549f, 0.82402458f,  1.29126623f,  4.67926869f,  0.15279416f,
    0.04732800f,  0.07147660f,  0.24818793f,  0.11946014f,  0.18403542f,
    0.21577589f,  0.17671133f,  -0.08234199f, 0.03522220f,  -0.15400573f,
    -0.01708496f, -0.10460475f, -0.05856720f, 0.10988067f,  0.13692078f,
    -0.44200069f, 0.04378286f,  -0.53302374f, -0.34864098f, -0.46814065f,
    -0.41584435f, 1.00049619f,  0.18438276f};
static const float SCALER_SCALE[48] = {
    0.00000000f,  1.11948895f,  2.66582324f,  2.75875460f,  0.61577876f,
    0.63005866f,  1.11948895f,  5.24600307f,  0.00000000f,  2.24265345f,
    5.28258264f,  5.29274689f,  1.25380305f,  1.25416601f,  2.24265345f,
    10.36550167f, 0.00000000f,  1.98977531f,  3.70032405f,  3.51277384f,
    1.33743717f,  1.33411043f,  1.98977531f,  6.48004915f,  0.16278660f,
    0.04732353f,  0.20187692f,  0.14988145f,  0.17309676f,  0.16454597f,
    0.08835064f,  0.17788873f,  0.07997004f,  0.03593683f,  0.11049219f,
    0.11446219f,  0.08192206f,  0.08453596f,  0.06048115f,  0.14184031f,
    0.89382048f,  0.06704045f,  0.85492860f,  0.95363405f,  0.88416397f,
    0.90578035f,  0.03526090f,  0.26373690f};

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
    /* Mean-center gyro axes (ch 0-2) per window to remove the per-sensor
       DC bias — must match the notebook's window_features(). Accel (3-5)
       keeps its DC component (gravity/tilt). */
    if (ch < 3) {
      float m = 0.0f;
      for (int t = 0; t < WINDOW_SIZE; t++)
        m += col[t];
      m /= WINDOW_SIZE;
      for (int t = 0; t < WINDOW_SIZE; t++)
        col[t] -= m;
    }
    channel_stats(col, WINDOW_SIZE, features + ch * N_STATS);
  }

  /* Normalise with StandardScaler and copy into model input. The centered
     gyro-mean features have zero variance (scale==0); feed 0 for them to
     avoid division by zero — they carry no information by construction. */
  for (int i = 0; i < N_FEATURES; i++)
    inp->data.f[i] = (SCALER_SCALE[i] > 1e-6f)
                         ? (features[i] - SCALER_MEAN[i]) / SCALER_SCALE[i]
                         : 0.0f;

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
