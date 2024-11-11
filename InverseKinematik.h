

#include "math.h"


int deltaCaltAngleYZ(float x0, float y0, float z0, float *delta);
int deltaCaltUbverse(float x0, float y0, float z0, float *delta1, float *delta2, float *delta3);
long getSteps(float delta);

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

int deltaCalcAngleQuat(float x0, float y0, float z0, float *delta);
int deltaCalcAnglesQ(Quaternion* q, float *delta1, float *delta2, float *delta3);
void getValue(Quaternion* q,float*x0,float* y0,float* z0);
