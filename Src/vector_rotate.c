#include "vector_rotate.h"

void rotateVector(double rotation[], double vector[]) {
    double x = TO_RAD(rotation[0]);
    double y = TO_RAD(rotation[1]);
    double z = TO_RAD(rotation[2]);
    double tempVector[] = { 0, 0, 9.8 };

    /* 
     * This might not work for all compilers. Not all of them support 
     * variables in initialization blocks. If it does not, move the computations 
     * out of the initialization block and leave only literals there or
     * try initializing everything dynamically. 
     */ 
    double S[] = { sin(x), sin(y), sin(z) };
    double C[] = { cos(x), cos(y), cos(z) };

    double rotationMatrix[][3][3] = {
        {
            { 1, 0, 0 },
            { 0, C[0], -S[0] },
            { 0, S[0], C[0] }
        }, 
        {
            { C[1], 0, S[1] },
            { 0, 1, 0 },
            { -S[1], 0, C[1] }
        },
        {
            { C[2], -S[2], 0 },
            { S[2], C[2], 0 },
            { 0, 0, 1}
        }
    };
    multiplyMatrices(rotationMatrix, 0, tempVector);
    multiplyMatrices(rotationMatrix, 1, tempVector);
    multiplyMatrices(rotationMatrix, 2, tempVector);
    copyArray(vector, tempVector, 3);
}

static inline void multiplyMatrices(double rotationMatrix[][3][3], int dim, double vector[3]) {
    double sum;
    double newVector[3];

    int i, j;

    for (i = 0; i < 3; i++) {
        sum = 0;
        for (j = 0; j < 3; j++) {
            sum += rotationMatrix[dim][i][j] * vector[j];   
        } 
        newVector[i] = sum;
    }
    copyArray(vector, newVector, 3);
}

static inline void copyArray(double* to, double* from, int len) {
    for (int i = 0; i < len; i++) {
        to[i] = from[i];
    }    
} 
