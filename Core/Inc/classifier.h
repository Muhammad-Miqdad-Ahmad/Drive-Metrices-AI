#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define CLASSIFIER_N_CLASSES 6

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

#ifdef __cplusplus
}
#endif
