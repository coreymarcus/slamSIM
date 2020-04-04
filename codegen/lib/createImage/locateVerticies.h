/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * locateVerticies.h
 *
 * Code generation for function 'locateVerticies'
 *
 */

#ifndef LOCATEVERTICIES_H
#define LOCATEVERTICIES_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "createImage_types.h"

/* Function Declarations */
extern void b_locateVerticies(const struct1_T C_faces[6], const double P[3],
  const double q[4], const double K[9], double Verts[4]);
extern void locateVerticies(const struct1_T C_faces[6], const double P[3], const
  double q[4], const double K[9], double Verts[4]);

#endif

/* End of code generation (locateVerticies.h) */
