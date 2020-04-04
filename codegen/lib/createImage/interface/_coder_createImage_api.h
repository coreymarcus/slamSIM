/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_createImage_api.h
 *
 * Code generation for function '_coder_createImage_api'
 *
 */

#ifndef _CODER_CREATEIMAGE_API_H
#define _CODER_CREATEIMAGE_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_createImage_api.h"

/* Type Definitions */
#ifndef typedef_struct1_T
#define typedef_struct1_T

typedef struct {
  real_T color[3];
  char_T plane;
  real_T center[3];
  real_T vertex[12];
} struct1_T;

#endif                                 /*typedef_struct1_T*/

#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T P[3];
  real_T s;
  real_T sw;
  real_T ec[3];
  struct1_T faces[6];
} struct0_T;

#endif                                 /*typedef_struct0_T*/

#ifndef typedef_struct2_T
#define typedef_struct2_T

typedef struct {
  real_T P[3];
  real_T s[3];
  real_T sw;
  real_T ec[3];
  boolean_T isAllEncomp;
  struct1_T faces[6];
} struct2_T;

#endif                                 /*typedef_struct2_T*/

#ifndef typedef_cell_0
#define typedef_cell_0

typedef struct {
  struct0_T f1;
  struct0_T f2;
  struct0_T f3;
  struct0_T f4;
  struct0_T f5;
  struct0_T f6;
  struct2_T f7;
  struct0_T f8;
  struct0_T f9;
  struct0_T f10;
  struct0_T f11;
} cell_0;

#endif                                 /*typedef_cell_0*/

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void createImage(cell_0 *C, real_T P[3], real_T q[4], real_T V[921600],
  real_T sz[2], real_T K[9], emxArray_real_T *I);
extern void createImage_api(const mxArray * const prhs[6], int32_T nlhs, const
  mxArray *plhs[1]);
extern void createImage_atexit(void);
extern void createImage_initialize(void);
extern void createImage_terminate(void);
extern void createImage_xil_terminate(void);

#endif

/* End of code generation (_coder_createImage_api.h) */
