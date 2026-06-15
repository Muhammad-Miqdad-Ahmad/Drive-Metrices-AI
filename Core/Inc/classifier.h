#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CLASSIFIER_N_CLASSES 6
#define CLASSIFIER_WINDOW    28   /* samples per inference window   */
#define CLASSIFIER_CHANNELS  6    /* GyroX,GyroY,GyroZ,AccX,AccY,AccZ */

extern const char *CLASS_NAMES[CLASSIFIER_N_CLASSES];

typedef struct {
    int   best;                        /* index of most probable class */
    float probs[CLASSIFIER_N_CLASSES]; /* softmax outputs              */
} Classifier_Result_t;

/* Load the TFLite model and allocate tensors. Returns 0 on success. */
int Classifier_Init(void);

/* Push one sample [GyroX, GyroY, GyroZ, AccX, AccY, AccZ] (dps / g).
   Returns 1 when a window completed and *res holds a new prediction,
   0 otherwise. */
int Classifier_Push(const float sample[6], Classifier_Result_t *res);

/* Copy the current window (oldest-first) into out as
   CLASSIFIER_WINDOW * CLASSIFIER_CHANNELS floats, laid out
   [sample0: gx,gy,gz,ax,ay,az][sample1: ...]...  These are the RAW
   values (same ordering the model uses, before gyro centering). */
void Classifier_CopyWindow(float *out);

#ifdef __cplusplus
}
#endif
