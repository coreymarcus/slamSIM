/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * strcmp.cpp
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "createImage.h"
#include "strcmp.h"

/* Function Definitions */
boolean_T b_strcmp(char a)
{
  boolean_T b_bool;
  b_bool = false;
  if (!(a != 'x')) {
    b_bool = true;
  }

  return b_bool;
}

boolean_T c_strcmp(char a)
{
  boolean_T b_bool;
  b_bool = false;
  if (!(a != 'y')) {
    b_bool = true;
  }

  return b_bool;
}

boolean_T d_strcmp(char a)
{
  boolean_T b_bool;
  b_bool = false;
  if (!(a != 'z')) {
    b_bool = true;
  }

  return b_bool;
}

/* End of code generation (strcmp.cpp) */
