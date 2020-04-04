/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "createImage.h"
#include "main.h"
#include "createImage_terminate.h"
#include "createImage_emxAPI.h"
#include "createImage_initialize.h"

/* Function Declarations */
static void argInit_1x2_real_T(double result[2]);
static void argInit_1x3_real_T(double result[3]);
static void argInit_3x1_real_T(double result[3]);
static void argInit_3x3_real_T(double result[9]);
static void argInit_3x4_real_T(double result[12]);
static void argInit_480x640x3_real_T(double result[921600]);
static void argInit_4x1_real_T(double result[4]);
static void argInit_6x1_struct1_T(struct1_T result[6]);
static boolean_T argInit_boolean_T();
static void argInit_cell_0(cell_0 *result);
static char argInit_char_T();
static double argInit_real_T();
static void argInit_struct0_T(struct0_T *result);
static void argInit_struct1_T(struct1_T *result);
static void argInit_struct2_T(struct2_T *result);
static void main_createImage();

/* Function Definitions */
static void argInit_1x2_real_T(double result[2])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static void argInit_1x3_real_T(double result[3])
{
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 3; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static void argInit_3x1_real_T(double result[3])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_3x3_real_T(double result[9])
{
  int idx0;
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 3; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_3x4_real_T(double result[12])
{
  int idx0;
  int idx1;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 4; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      result[idx0 + 3 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_480x640x3_real_T(double result[921600])
{
  int idx0;
  int idx1;
  int idx2;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 480; idx0++) {
    for (idx1 = 0; idx1 < 640; idx1++) {
      for (idx2 = 0; idx2 < 3; idx2++) {
        /* Set the value of the array element.
           Change this value to the value that the application requires. */
        result[(idx0 + 480 * idx1) + 307200 * idx2] = argInit_real_T();
      }
    }
  }
}

static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 4; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_6x1_struct1_T(struct1_T result[6])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    argInit_struct1_T(&result[idx0]);
  }
}

static boolean_T argInit_boolean_T()
{
  return false;
}

static void argInit_cell_0(cell_0 *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  argInit_struct0_T(&result->f1);
  argInit_struct0_T(&result->f2);
  argInit_struct0_T(&result->f3);
  argInit_struct0_T(&result->f4);
  argInit_struct0_T(&result->f5);
  argInit_struct0_T(&result->f6);
  argInit_struct2_T(&result->f7);
  argInit_struct0_T(&result->f8);
  argInit_struct0_T(&result->f9);
  argInit_struct0_T(&result->f10);
  argInit_struct0_T(&result->f11);
}

static char argInit_char_T()
{
  return '?';
}

static double argInit_real_T()
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  argInit_3x1_real_T(result->P);
  result->s = argInit_real_T();
  result->sw = argInit_real_T();
  argInit_1x3_real_T(result->ec);
  argInit_6x1_struct1_T(result->faces);
}

static void argInit_struct1_T(struct1_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  argInit_1x3_real_T(result->color);
  result->plane = argInit_char_T();
  argInit_3x1_real_T(result->center);
  argInit_3x4_real_T(result->vertex);
}

static void argInit_struct2_T(struct2_T *result)
{
  /* Set the value of each structure field.
     Change this value to the value that the application requires. */
  argInit_3x1_real_T(result->P);
  argInit_3x1_real_T(result->s);
  result->sw = argInit_real_T();
  argInit_1x3_real_T(result->ec);
  result->isAllEncomp = argInit_boolean_T();
  argInit_6x1_struct1_T(result->faces);
}

static void main_createImage()
{
  emxArray_real_T *I;
  cell_0 r1;
  double dv4[3];
  double dv5[4];
  static double dv6[921600];
  double dv7[2];
  double dv8[9];
  emxInitArray_real_T(&I, 3);

  /* Initialize function 'createImage' input arguments. */
  /* Initialize function input argument 'C'. */
  /* Initialize function input argument 'P'. */
  /* Initialize function input argument 'q'. */
  /* Initialize function input argument 'V'. */
  /* Initialize function input argument 'sz'. */
  /* Initialize function input argument 'K'. */
  /* Call the entry-point 'createImage'. */
  argInit_cell_0(&r1);
  argInit_3x1_real_T(dv4);
  argInit_4x1_real_T(dv5);
  argInit_480x640x3_real_T(dv6);
  argInit_1x2_real_T(dv7);
  argInit_3x3_real_T(dv8);
  createImage(&r1, dv4, dv5, dv6, dv7, dv8, I);
  emxDestroyArray_real_T(I);
}

int main(int, const char * const [])
{
  /* Initialize the application.
     You do not need to do this more than one time. */
  createImage_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_createImage();

  /* Terminate the application.
     You do not need to do this more than one time. */
  createImage_terminate();
  return 0;
}

/* End of code generation (main.cpp) */
