//
//  main.c
//  Inverse_Kineamtik_V5.6
// Calcutlate ZY Ebene, dann Koordinatentransformation
//  Created by Cenk Orhan.
//

#include <stdio.h>
#include "InverseKinematik.h"
#include <math.h>


#include <stdlib.h>
#define PI 3.141592

void kreis(float** mat, float higth, float radius);
float** newMat(int row, int col);
void helix(float** mat, float higth, float radius);



float **newMat(int row, int col);
void kreis(float **mat, float higth, float radius);


    int main(void) {
        // Kartesische Koordinaten
        float x = 80;
        float y = 80;
        float z = -400;

        //WINKEL
        float delta1 = 0;
        float delta2 = 0;
        float delta3 = 0;
        float delta4 = 0;

        float x1 = 0;
        float y1 = 0;
        float z1 = 0;

        float *pX = &x1;
        float *pY = &y1;
        float *pZ = &z1;

        float *pDelta1 = &delta1;
        float *pDelta2 = &delta2;
        float *pDelta3 = &delta3;
        float *pDelta4 = &delta4;


        int row, col;
        row = 50;
        col = 3;

        int test = deltaCaltUbverse(x,y,z,pDelta1,pDelta2,pDelta3);
        printf("delta1: %9.6f\n", delta1);
        printf("delta2: %9.6f\n", delta2);
        printf("delta3: %9.6f\n", delta3);
        float** mat = newMat(row,col);
        float** wink = newMat(row,col);
        kreis(mat,-330.0, 50.0);

        Quaternion q;
        q.z =  0.000000456;
        q.x =   0.0554;
        q.w =   0.00424;
        q.y =  0.000000000000456;

        getValue(&q,pX,pY,pZ);
        printf("x: %9.6f\n", x1);
        printf("y: %9.6f\n", y1);
        printf("z: %9.6f\n", z1);

        int test3 = deltaCalcAngleQuat(x1,y1,z1,pDelta4);

        printf("delta1: %9.6f\n", delta4);








        return 0;
    }

void kreis(float** mat, float higth, float radius)
{
    int j = 0;


    for (float i = 0; i <= 2 * PI; i = i + 0.04*PI)
    {

        mat[j][0] = radius * cosf(i); //x
        mat[j][1] = radius * sinf(i); //y
        mat[j][2] = higth;            //z
        j++;

    }


}


void helix(float** mat, float higth, float radius)
{
    int j = 0;

    for (float i = 0; i <= 2 * PI; i = i + 0.01*PI)
    {

        mat[j][0] = radius * cosf(i); //x
        mat[j][1] = radius * sinf(i); //y
        mat[j][2] = higth + j * -0.6;  //z
        j++;

    }


}


float** newMat(int row, int col)
{
    float** mat = malloc(row * sizeof(float*));
    for (int i = 0; i < row; i++)
    {
        mat[i] =  malloc(col * sizeof(float));
    }
    return mat;


}





