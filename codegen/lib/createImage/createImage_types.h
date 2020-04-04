/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * createImage_types.h
 *
 * Code generation for function 'createImage'
 *
 */

#ifndef CREATEIMAGE_TYPES_H
#define CREATEIMAGE_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
typedef struct {
  double color[3];
  char plane;
  double center[3];
  double vertex[12];
} struct1_T;

typedef struct {
  double P[3];
  double s;
  double sw;
  double ec[3];
  struct1_T faces[6];
} struct0_T;

typedef struct {
  double P[3];
  double s[3];
  double sw;
  double ec[3];
  boolean_T isAllEncomp;
  struct1_T faces[6];
} struct2_T;

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

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

typedef struct {
  emxArray_real_T *f1;
} cell_wrap_1;

#ifndef struct_sYTAMIa09ytPxnAg0RrY4uC_tag
#define struct_sYTAMIa09ytPxnAg0RrY4uC_tag

struct sYTAMIa09ytPxnAg0RrY4uC_tag
{
  emxArray_real_T *f1;
};

#endif                                 /*struct_sYTAMIa09ytPxnAg0RrY4uC_tag*/

typedef sYTAMIa09ytPxnAg0RrY4uC_tag cell_wrap_2;

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_boolean_T*/

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int32_T*/

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int8_T*/
#endif

/* End of code generation (createImage_types.h) */
