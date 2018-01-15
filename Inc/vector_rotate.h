#ifndef VECTOR_ROTATE
#define VECTOR_ROTATE

#include <math.h>
#include <stdio.h>

#define PI 3.14
#define TO_RAD(ang) ((ang) * 180) / PI

void rotateVector(double rotation[], double vector []); 

static inline void multiplyMatrices(double rotationMatrix[][3][3], int dim, double vector[3]);
static inline void copyArray(double* to, double* from, int len);

#endif
