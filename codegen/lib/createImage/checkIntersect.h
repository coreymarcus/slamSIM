/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkIntersect.h
 *
 * Code generation for function 'checkIntersect'
 *
 */

#ifndef CHECKINTERSECT_H
#define CHECKINTERSECT_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "createImage_types.h"

/* Function Declarations */
extern void b_checkIntersect(double C_sw, const struct1_T C_faces[6], const
  double P[3], const double v[3], double *I, double *D);
extern void checkIntersect(double C_sw, const struct1_T C_faces[6], const double
  P[3], const double v[3], double *I, double *D);

#endif

/* End of code generation (checkIntersect.h) */
