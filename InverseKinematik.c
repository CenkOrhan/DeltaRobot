//
// Created by Cenk Orhan on 14.10.22.
//
//#include <math.h>
#include "InverseKinematik.h"

const float e = 21.03;   // end effector
const float f = 112.77; // base
const float re = 347.0;   //upper arm
const float rf = 129.87; //lower arm
const float i = 1;   // gearratio


// trigonometric constants
const float PI = 3.141592653;       // PI
const float sin120 = 0.866025403;   //sqrt3/2.0;
const float cos120 = -0.5;
// robot geometry
// (look at pics above for explanation)

int deltaCaltAngleYZ(float x0, float y0, float z0, float *delta)
{
    float y1 = -0.5 * 0.57735 * f;
    y0 -= 0.5 * 0.57735 * e; //f2 * tg30
    float a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1)/(2*z0);
    float b = (y1-y0)/z0;
    //diskriminate
    float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
    if(d<0) return -1; //Punkt exestiert nicht
    //Check einbauen ob Wert im Wertebereich?
    float yj = (y1 - a * b - sqrtf(d)) / (b * b + 1);
    float zj = a + b*yj;
    *delta = ((float)180.0 * atan(-zj/(y1-yj))/PI + ((yj>y1)?180.0:0.0))*i;
    return 0;
}

int deltaCaltUbverse(float x0, float y0, float z0, float *delta1, float *delta2, float *delta3)
{
    *delta1 = *delta2 = *delta3 = 0;
    int status = deltaCaltAngleYZ( x0, y0, z0, delta1);

    if (status == 0) {   // rotate coords to +120 deg
        status = deltaCaltAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, delta2);
    }
    if (status == 0) // rotate coords to -120 deg
        status = deltaCaltAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, delta3);
    return status;
}

long getSteps(float delta)
{
    float steps = 4.44 * delta;
    return (long) steps;
}

int deltaCalcAnglesQ(Quaternion* q, float *delta1, float *delta2, float *delta3) {
    *delta1 = *delta2 = *delta3 = 0;

    // Convert quaternion to rotation matrix
    float R[3][3];
    R[0][0] = 1 - 2 * (q->y * q->y + q->z * q->z);
    R[0][1] = 2 * (q->x * q->y - q->w * q->z);
    R[0][2] = 2 * (q->x * q->z + q->w * q->y);
    R[1][0] = 2 * (q->x * q->y + q->w * q->z);
    R[1][1] = 1 - 2 * (q->x * q->x + q->z * q->z);
    R[1][2] = 2 * (q->y * q->z - q->w * q->x);
    R[2][0] = 2 * (q->x * q->z - q->w * q->y);
    R[2][1] = 2 * (q->y * q->z + q->w * q->x);
    R[2][2] = 1 - 2 * (q->x * q->x + q->y * q->y);

    // Calculate position of end effector
    float x0 = R[0][2] * (-e - f);
    float y0 = R[1][2] * (-e - f);
    float z0 = R[2][2] * (-e - f);

    // Calculate delta angles using inverse kinematics
    int status = deltaCalcAngleQuat(x0, y0, z0, delta1);

    if (status == 0) {
        status = deltaCalcAngleQuat(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, delta2);
    }

    if (status == 0) {
        status = deltaCalcAngleQuat(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, delta3);
    }

    return status;
}

int deltaCalcAngleQuat(float x0, float y0, float z0, float *delta) {
    float y1 = -0.5 * 0.57735 * f;
    y0 -= 0.5 * 0.57735 * e;
    float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
    float b = (y1 - y0) / z0;
    float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
    return 0;
}

void getValue(Quaternion* q,float*x0,float* y0,float* z0)
{
    float R[3][3];
    R[0][0] = 1 - 2 * (q->y * q->y + q->z * q->z);
    R[0][1] = 2 * (q->x * q->y - q->w * q->z);
    R[0][2] = 2 * (q->x * q->z + q->w * q->y);
    R[1][0] = 2 * (q->x * q->y + q->w * q->z);
    R[1][1] = 1 - 2 * (q->x * q->x + q->z * q->z);
    R[1][2] = 2 * (q->y * q->z - q->w * q->x);
    R[2][0] = 2 * (q->x * q->z - q->w * q->y);
    R[2][1] = 2 * (q->y * q->z + q->w * q->x);
    R[2][2] = 1 - 2 * (q->x * q->x + q->y * q->y);

    // Calculate position of end effector
    *x0 = R[0][2] * (-e - f);
    *y0 = R[1][2] * (-e - f);
    *z0 = R[2][2] * (-e - f);
}

