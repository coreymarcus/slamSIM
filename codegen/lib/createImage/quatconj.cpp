/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * quatconj.cpp
 *
 * Code generation for function 'quatconj'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "createImage.h"
#include "quatconj.h"

/* Function Definitions */
void quatconj(double q[4])
{
  int k;
  for (k = 0; k < 3; k++) {
    q[k + 1] = -q[k + 1];
  }
}

/* End of code generation (quatconj.cpp) */
