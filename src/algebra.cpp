/*
  algebra.hpp: This file contains a number of utilities useful for handling
  3D vectors

  This work is an adaptation from vvector.h, written by Linas Vepstras. The
  original code can be found at:

  https://github.com/markkilgard/glut/blob/master/lib/gle/vvector.h

  HISTORY:
  Written by Linas Vepstas, August 1991
  Added 2D code, March 1993
  Added Outer products, C++ proofed, Linas Vepstas October 1993
  Adapted for altitude estimation tasks by Juan Gallostra June 2018
*/

#include <cmath>
#include <stdio.h>

#include "algebra.h"

// Copy 3D vector
void copyVector(float b[3],float a[3])
{
   b[0] = a[0];
   b[1] = a[1];
   b[2] = a[2];
}


// Vector difference
void subtractVectors(float v21[3], float v2[3], float v1[3])
{
   v21[0] = v2[0] - v1[0];
   v21[1] = v2[1] - v1[1];
   v21[2] = v2[2] - v1[2];
}

// Vector sum
void sumVectors(float v21[3], float v2[3], float v1[3])
{
   v21[0] = v2[0] + v1[0];
   v21[1] = v2[1] + v1[1];
   v21[2] = v2[2] + v1[2];
}

// scalar times vector
void scaleVector(float c[3],float a, float b[3])
{
   (c)[0] = a*b[0];
   (c)[1] = a*b[1];
   (c)[2] = a*b[2];
}

// accumulate scaled vector
void accumulateScaledVector(float c[3], float a, float b[3])
{
   (c)[0] += a*b[0];
   (c)[1] += a*b[1];
   (c)[2] += a*b[2];
}

// Vector dot product
void dotProductVectors(float * c, float a[3], float b[3])
{
   *c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Vector length
void vectorLength(float * len, float a[3])
{
   float tmp;
   tmp = a[0]*a[0] + a[1]*a[1]+a[2]*a[2];
   *len = sqrt(tmp);
}

// Normalize vector
void normalizeVector(float a[3])
{
   float len;
   vectorLength(& len,a);
   if (len != 0.0) {
      len = 1.0 / len;
      a[0] *= len;
      a[1] *= len;
      a[2] *= len;
   }
}

// 3D Vector cross product yeilding vector
void crossProductVectors(float c[3], float a[3], float b[3])
{
   c[0] = a[1] * b[2] - a[2] * b[1];
   c[1] = a[2] * b[0] - a[0] * b[2];
   c[2] = a[0] * b[1] - a[1] * b[0];
}

// initialize matrix
void identityMatrix3x3(float m[3][3])
{
   m[0][0] = 1.0;
   m[0][1] = 0.0;
   m[0][2] = 0.0;

   m[1][0] = 0.0;
   m[1][1] = 1.0;
   m[1][2] = 0.0;

   m[2][0] = 0.0;
   m[2][1] = 0.0;
   m[2][2] = 1.0;
}

// matrix copy
void copyMatrix3x3(float b[3][3], float a[3][3])
{
   b[0][0] = a[0][0];
   b[0][1] = a[0][1];
   b[0][2] = a[0][2];

   b[1][0] = a[1][0];
   b[1][1] = a[1][1];
   b[1][2] = a[1][2];

   b[2][0] = a[2][0];
   b[2][1] = a[2][1];
   b[2][2] = a[2][2];
}

// matrix transpose
void transposeMatrix3x3(float b[3][3], float a[3][3])
{
   b[0][0] = a[0][0];
   b[0][1] = a[1][0];
   b[0][2] = a[2][0];

   b[1][0] = a[0][1];
   b[1][1] = a[1][1];
   b[1][2] = a[2][1];

   b[2][0] = a[0][2];
   b[2][1] = a[1][2];
   b[2][2] = a[2][2];
}

// multiply matrix by scalar
void scaleMatrix3x3(float b[3][3], float s, float a[3][3])
{
   b[0][0] = (s) * a[0][0];
   b[0][1] = (s) * a[0][1];
   b[0][2] = (s) * a[0][2];

   b[1][0] = (s) * a[1][0];
   b[1][1] = (s) * a[1][1];
   b[1][2] = (s) * a[1][2];

   b[2][0] = (s) * a[2][0];
   b[2][1] = (s) * a[2][1];
   b[2][2] = (s) * a[2][2];
}

// multiply matrix by scalar and add result to another matrix
void scaleAndAccumulateMatrix3x3(float b[3][3], float s, float a[3][3])
{
   b[0][0] += s * a[0][0];
   b[0][1] += s * a[0][1];
   b[0][2] += s * a[0][2];

   b[1][0] += s * a[1][0];
   b[1][1] += s * a[1][1];
   b[1][2] += s * a[1][2];

   b[2][0] += s * a[2][0];
   b[2][1] += s * a[2][1];
   b[2][2] += s * a[2][2];
}

// matrix product
// c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y]
void matrixProduct3x3(float c[3][3], float a[3][3], float b[3][3])
{
   c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0];
   c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1];
   c[0][2] = a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2];

   c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0];
   c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1];
   c[1][2] = a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2];

   c[2][0] = a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0];
   c[2][1] = a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1];
   c[2][2] = a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2];
}

// matrix times vector
void matrixDotVector3x3(float p[3], float m[3][3], float v[3])
{
   p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
   p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
   p[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}

// determinant of matrix
// Computes determinant of matrix m, returning d
void determinant3x3(float * d, float m[3][3])
{
   *d = m[0][0] * (m[1][1]*m[2][2] - m[1][2] * m[2][1]);
   *d -= m[0][1] * (m[1][0]*m[2][2] - m[1][2] * m[2][0]);
   *d += m[0][2] * (m[1][0]*m[2][1] - m[1][1] * m[2][0]);
}

// adjoint of matrix
// Computes adjoint of matrix m, returning a
// (Note that adjoint is just the transpose of the cofactor matrix)
void adjoint3x3(float a[3][3], float m[3][3])
{
   a[0][0] = m[1][1]*m[2][2] - m[1][2]*m[2][1];
   a[1][0] = - (m[1][0]*m[2][2] - m[2][0]*m[1][2]);
   a[2][0] = m[1][0]*m[2][1] - m[1][1]*m[2][0];
   a[0][1] = - (m[0][1]*m[2][2] - m[0][2]*m[2][1]);
   a[1][1] = m[0][0]*m[2][2] - m[0][2]*m[2][0];
   a[2][1] = - (m[0][0]*m[2][1] - m[0][1]*m[2][0]);
   a[0][2] = m[0][1]*m[1][2] - m[0][2]*m[1][1];
   a[1][2] = - (m[0][0]*m[1][2] - m[0][2]*m[1][0]);
   a[2][2] = m[0][0]*m[1][1] - m[0][1]*m[1][0];
}

// compute adjoint of matrix and scale
// Computes adjoint of matrix m, scales it by s, returning a
void scaleAdjoint3x3(float a[3][3], float s, float m[3][3])
{
   a[0][0] = (s) * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
   a[1][0] = (s) * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);
   a[2][0] = (s) * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

   a[0][1] = (s) * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);
   a[1][1] = (s) * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);
   a[2][1] = (s) * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);

   a[0][2] = (s) * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
   a[1][2] = (s) * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);
   a[2][2] = (s) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);
}

// inverse of matrix
// Compute inverse of matrix a, returning determinant m and
// inverse b
void invert3x3(float b[3][3], float a[3][3])
{
   float tmp;
   determinant3x3(& tmp, a);
   tmp = 1.0 / (tmp);
   scaleAdjoint3x3(b, tmp, a);
}

// skew matrix from vector
void skew(float a[3][3], float v[3])
{
  a[0][1] = -v[2];
  a[0][2] = v[1];
  a[1][2] = -v[0];
  a[1][0] = v[2];
  a[2][0] = -v[1];
  a[2][1] = v[0];
  // set diagonal to 0
  a[0][0] = 0.0;
  a[1][1] = 0.0;
  a[2][2] = 0.0;
}

void printMatrix3X3(float mmm[3][3])
{
   int i,j;
   printf ("matrix mmm is \n");
   if (mmm == NULL) {
      printf (" Null \n");
   } else {
      for (i=0; i<3; i++) {
         for (j=0; j<3; j++) {
            printf ("%f ", mmm[i][j]);
         }
         printf (" \n");
      }
   }
}

void vecPrint(float a[3])
{
   float len;
   vectorLength(& len, a);
   printf(" a is %f %f %f length of a is %f \n", a[0], a[1], a[2], len);
}
