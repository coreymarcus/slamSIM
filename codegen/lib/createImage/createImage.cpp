/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * createImage.cpp
 *
 * Code generation for function 'createImage'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "createImage.h"
#include "createImage_emxutil.h"
#include "checkIntersect.h"
#include "quat2dcmCoder.h"
#include "quatconj.h"
#include "locateVerticies.h"

/* Function Definitions */
void createImage(const cell_0 *C, const double P[3], const double q[4], const
                 double V[921600], const double sz[2], const double K[9],
                 emxArray_real_T *I)
{
  int i0;
  double bounds[44];
  double Verts[4];
  double xMin;
  double xMax;
  double yMin;
  double yMax;
  emxArray_int8_T *Itemplate;
  int idx;
  emxArray_real_T *Iloop;
  int nm1d2;
  int i1;
  emxArray_real_T *xIdxs;
  emxArray_real_T *yIdxs;
  double u1;
  int ii;
  emxArray_real_T *tempCol;
  emxArray_int32_T *targRow;
  emxArray_int32_T *r0;
  emxArray_boolean_T *x;
  emxArray_int32_T *b_ii;
  cell_wrap_2 reshapes[2];
  int nx;
  cell_wrap_1 Icell[11];
  boolean_T empty_non_axis_sizes;
  int i2;
  int jj;
  cell_wrap_2 b_reshapes[2];
  double dv0[9];
  double dv1[3];
  cell_wrap_2 c_reshapes[2];
  boolean_T exitg1;
  cell_wrap_2 d_reshapes[2];
  cell_wrap_2 e_reshapes[2];
  cell_wrap_2 f_reshapes[2];
  cell_wrap_2 g_reshapes[2];
  cell_wrap_2 h_reshapes[2];
  cell_wrap_2 i_reshapes[2];
  cell_wrap_2 j_reshapes[2];
  cell_wrap_2 k_reshapes[2];

  /* creates an RGB image of a cube with a white background given cube */
  /* parameters, camera position and orientation, and camera calibration matrix */
  /*  Inputs */
  /*  C = [Nx1] cell array of cube structures */
  /*  P = [3x1] camera position in inertial frame */
  /*  q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts */
  /*  V = [height x width x 3] matrix of vectors corresponding to each pixels */
  /*  sz = [2x1] = [width, height] image size in pixels */
  /*   */
  /*  Outputs */
  /*  I = [height x width x 4] RGB+D triplet information */
  /* get number of cubes */
  /* initialize image */
  /* find the verticies of all the cubes */
  for (i0 = 0; i0 < 44; i0++) {
    bounds[i0] = 1.0;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f1.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[0] = xMin;
    bounds[11] = xMax;
    bounds[22] = yMin;
    bounds[33] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f2.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[1] = xMin;
    bounds[12] = xMax;
    bounds[23] = yMin;
    bounds[34] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f3.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[2] = xMin;
    bounds[13] = xMax;
    bounds[24] = yMin;
    bounds[35] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f4.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[3] = xMin;
    bounds[14] = xMax;
    bounds[25] = yMin;
    bounds[36] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f5.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[4] = xMin;
    bounds[15] = xMax;
    bounds[26] = yMin;
    bounds[37] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f6.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[5] = xMin;
    bounds[16] = xMax;
    bounds[27] = yMin;
    bounds[38] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  b_locateVerticies(C->f7.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[6] = xMin;
    bounds[17] = xMax;
    bounds[28] = yMin;
    bounds[39] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f8.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[7] = xMin;
    bounds[18] = xMax;
    bounds[29] = yMin;
    bounds[40] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f9.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[8] = xMin;
    bounds[19] = xMax;
    bounds[30] = yMin;
    bounds[41] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f10.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[9] = xMin;
    bounds[20] = xMax;
    bounds[31] = yMin;
    bounds[42] = yMax;
  }

  /* locate the verticies of the cube in pixel form */
  locateVerticies(C->f11.faces, P, q, K, Verts);
  if ((Verts[0] < 1.0) || rtIsNaN(Verts[0])) {
    xMin = 1.0;
  } else {
    xMin = Verts[0];
  }

  if ((Verts[1] > sz[0]) || (rtIsNaN(Verts[1]) && (!rtIsNaN(sz[0])))) {
    xMax = sz[0];
  } else {
    xMax = Verts[1];
  }

  if ((Verts[2] < 1.0) || rtIsNaN(Verts[2])) {
    yMin = 1.0;
  } else {
    yMin = Verts[2];
  }

  if ((Verts[3] > sz[1]) || (rtIsNaN(Verts[3]) && (!rtIsNaN(sz[1])))) {
    yMax = sz[1];
  } else {
    yMax = Verts[3];
  }

  /* continue if the box is out of the frame */
  if ((yMax < 1.0) || (xMax < 1.0) || (xMin >= sz[0]) || (yMin >= sz[1])) {
  } else {
    bounds[10] = xMin;
    bounds[21] = xMax;
    bounds[32] = yMin;
    bounds[43] = yMax;
  }

  emxInit_int8_T(&Itemplate, 3);

  /*  cycle throught all the cubes and all the points */
  i0 = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  Itemplate->size[0] = (int)sz[1];
  Itemplate->size[1] = (int)sz[0];
  Itemplate->size[2] = 4;
  emxEnsureCapacity_int8_T(Itemplate, i0);
  idx = ((int)sz[1] * (int)sz[0]) << 2;
  for (i0 = 0; i0 < idx; i0++) {
    Itemplate->data[i0] = 1;
  }

  idx = (int)sz[0];
  for (i0 = 0; i0 < idx; i0++) {
    nm1d2 = (int)sz[1];
    for (i1 = 0; i1 < nm1d2; i1++) {
      Itemplate->data[(i1 + Itemplate->size[0] * i0) + Itemplate->size[0] *
        Itemplate->size[1] * 3] = 0;
    }
  }

  emxInit_real_T(&Iloop, 3);

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  emxInit_real_T1(&xIdxs, 2);
  if (rtIsNaN(bounds[0]) || rtIsNaN(bounds[11])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[11] < bounds[0]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[0]) || rtIsInf(bounds[11])) && (bounds[0] ==
              bounds[11])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[0]) == bounds[0]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[11] - bounds[0]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[11] - bounds[0]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[0] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[11] - bounds[0]) + 0.5);
    xMax = bounds[0] + xMin;
    yMin = xMax - bounds[11];
    yMax = bounds[0];
    u1 = bounds[11];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[11];
    } else if (yMin > 0.0) {
      xMax = bounds[0] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[0];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[0] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[0] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[0] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  emxInit_real_T1(&yIdxs, 2);
  if (rtIsNaN(bounds[22]) || rtIsNaN(bounds[33])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[33] < bounds[22]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[22]) || rtIsInf(bounds[33])) && (bounds[22] ==
              bounds[33])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[22]) == bounds[22]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[33] - bounds[22]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[33] - bounds[22]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[22] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[33] - bounds[22]) + 0.5);
    xMax = bounds[22] + xMin;
    yMin = xMax - bounds[33];
    yMax = bounds[22];
    u1 = bounds[33];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[33];
    } else if (yMin > 0.0) {
      xMax = bounds[22] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[22];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[22] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[22] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[22] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInit_real_T1(&tempCol, 2);
  emxInit_int32_T(&targRow, 2);
  emxInit_int32_T1(&r0, 1);
  emxInit_boolean_T(&x, 2);
  emxInit_int32_T(&b_ii, 2);
  emxInitMatrix_cell_wrap_2(reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
    reshapes[0].f1->size[0] = nx;
    reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
    reshapes[1].f1->size[0] = nx;
    reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = reshapes[0].f1->size[0];
    tempCol->size[1] = reshapes[0].f1->size[1] + reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = reshapes[0].f1->data[i2 +
          reshapes[0].f1->size[0] * i1];
      }
    }

    idx = reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + reshapes[0].f1->size[1])] =
          reshapes[1].f1->data[i2 + reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f1.sw, C->f1.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f1.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f1.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(reshapes);
  emxInitMatrix_cell_wrap_1(Icell);

  /* add this cube's image to the cell */
  i0 = Icell[0].f1->size[0] * Icell[0].f1->size[1] * Icell[0].f1->size[2];
  Icell[0].f1->size[0] = Iloop->size[0];
  Icell[0].f1->size[1] = Iloop->size[1];
  Icell[0].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[0].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[0].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[1]) || rtIsNaN(bounds[12])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[12] < bounds[1]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[1]) || rtIsInf(bounds[12])) && (bounds[1] ==
              bounds[12])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[1]) == bounds[1]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[12] - bounds[1]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[12] - bounds[1]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[1] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[12] - bounds[1]) + 0.5);
    xMax = bounds[1] + xMin;
    yMin = xMax - bounds[12];
    yMax = bounds[1];
    u1 = bounds[12];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[12];
    } else if (yMin > 0.0) {
      xMax = bounds[1] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[1];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[1] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[1] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[1] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[23]) || rtIsNaN(bounds[34])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[34] < bounds[23]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[23]) || rtIsInf(bounds[34])) && (bounds[23] ==
              bounds[34])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[23]) == bounds[23]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[34] - bounds[23]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[34] - bounds[23]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[23] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[34] - bounds[23]) + 0.5);
    xMax = bounds[23] + xMin;
    yMin = xMax - bounds[34];
    yMax = bounds[23];
    u1 = bounds[34];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[34];
    } else if (yMin > 0.0) {
      xMax = bounds[23] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[23];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[23] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[23] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[23] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(b_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = b_reshapes[0].f1->size[0] * b_reshapes[0].f1->size[1];
    b_reshapes[0].f1->size[0] = nx;
    b_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(b_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      b_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = b_reshapes[1].f1->size[0] * b_reshapes[1].f1->size[1];
    b_reshapes[1].f1->size[0] = nx;
    b_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(b_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      b_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = b_reshapes[0].f1->size[0];
    tempCol->size[1] = b_reshapes[0].f1->size[1] + b_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = b_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = b_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = b_reshapes[0].f1->data[i2 +
          b_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = b_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = b_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + b_reshapes[0].f1->size[1])] =
          b_reshapes[1].f1->data[i2 + b_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f2.sw, C->f2.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f2.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f2.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(b_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[1].f1->size[0] * Icell[1].f1->size[1] * Icell[1].f1->size[2];
  Icell[1].f1->size[0] = Iloop->size[0];
  Icell[1].f1->size[1] = Iloop->size[1];
  Icell[1].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[1].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[1].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[2]) || rtIsNaN(bounds[13])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[13] < bounds[2]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[2]) || rtIsInf(bounds[13])) && (bounds[2] ==
              bounds[13])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[2]) == bounds[2]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[13] - bounds[2]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[13] - bounds[2]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[2] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[13] - bounds[2]) + 0.5);
    xMax = bounds[2] + xMin;
    yMin = xMax - bounds[13];
    yMax = bounds[2];
    u1 = bounds[13];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[13];
    } else if (yMin > 0.0) {
      xMax = bounds[2] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[2];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[2] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[2] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[2] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[24]) || rtIsNaN(bounds[35])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[35] < bounds[24]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[24]) || rtIsInf(bounds[35])) && (bounds[24] ==
              bounds[35])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[24]) == bounds[24]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[35] - bounds[24]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[35] - bounds[24]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[24] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[35] - bounds[24]) + 0.5);
    xMax = bounds[24] + xMin;
    yMin = xMax - bounds[35];
    yMax = bounds[24];
    u1 = bounds[35];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[35];
    } else if (yMin > 0.0) {
      xMax = bounds[24] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[24];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[24] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[24] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[24] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(c_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = c_reshapes[0].f1->size[0] * c_reshapes[0].f1->size[1];
    c_reshapes[0].f1->size[0] = nx;
    c_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(c_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      c_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = c_reshapes[1].f1->size[0] * c_reshapes[1].f1->size[1];
    c_reshapes[1].f1->size[0] = nx;
    c_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(c_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      c_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = c_reshapes[0].f1->size[0];
    tempCol->size[1] = c_reshapes[0].f1->size[1] + c_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = c_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = c_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = c_reshapes[0].f1->data[i2 +
          c_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = c_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = c_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + c_reshapes[0].f1->size[1])] =
          c_reshapes[1].f1->data[i2 + c_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f3.sw, C->f3.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f3.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f3.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(c_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[2].f1->size[0] * Icell[2].f1->size[1] * Icell[2].f1->size[2];
  Icell[2].f1->size[0] = Iloop->size[0];
  Icell[2].f1->size[1] = Iloop->size[1];
  Icell[2].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[2].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[2].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[3]) || rtIsNaN(bounds[14])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[14] < bounds[3]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[3]) || rtIsInf(bounds[14])) && (bounds[3] ==
              bounds[14])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[3]) == bounds[3]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[14] - bounds[3]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[14] - bounds[3]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[3] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[14] - bounds[3]) + 0.5);
    xMax = bounds[3] + xMin;
    yMin = xMax - bounds[14];
    yMax = bounds[3];
    u1 = bounds[14];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[14];
    } else if (yMin > 0.0) {
      xMax = bounds[3] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[3];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[3] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[3] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[3] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[25]) || rtIsNaN(bounds[36])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[36] < bounds[25]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[25]) || rtIsInf(bounds[36])) && (bounds[25] ==
              bounds[36])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[25]) == bounds[25]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[36] - bounds[25]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[36] - bounds[25]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[25] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[36] - bounds[25]) + 0.5);
    xMax = bounds[25] + xMin;
    yMin = xMax - bounds[36];
    yMax = bounds[25];
    u1 = bounds[36];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[36];
    } else if (yMin > 0.0) {
      xMax = bounds[25] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[25];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[25] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[25] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[25] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(d_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = d_reshapes[0].f1->size[0] * d_reshapes[0].f1->size[1];
    d_reshapes[0].f1->size[0] = nx;
    d_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(d_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      d_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = d_reshapes[1].f1->size[0] * d_reshapes[1].f1->size[1];
    d_reshapes[1].f1->size[0] = nx;
    d_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(d_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      d_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = d_reshapes[0].f1->size[0];
    tempCol->size[1] = d_reshapes[0].f1->size[1] + d_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = d_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = d_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = d_reshapes[0].f1->data[i2 +
          d_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = d_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = d_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + d_reshapes[0].f1->size[1])] =
          d_reshapes[1].f1->data[i2 + d_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f4.sw, C->f4.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f4.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f4.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(d_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[3].f1->size[0] * Icell[3].f1->size[1] * Icell[3].f1->size[2];
  Icell[3].f1->size[0] = Iloop->size[0];
  Icell[3].f1->size[1] = Iloop->size[1];
  Icell[3].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[3].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[3].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[4]) || rtIsNaN(bounds[15])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[15] < bounds[4]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[4]) || rtIsInf(bounds[15])) && (bounds[4] ==
              bounds[15])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[4]) == bounds[4]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[15] - bounds[4]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[15] - bounds[4]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[4] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[15] - bounds[4]) + 0.5);
    xMax = bounds[4] + xMin;
    yMin = xMax - bounds[15];
    yMax = bounds[4];
    u1 = bounds[15];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[15];
    } else if (yMin > 0.0) {
      xMax = bounds[4] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[4];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[4] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[4] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[4] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[26]) || rtIsNaN(bounds[37])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[37] < bounds[26]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[26]) || rtIsInf(bounds[37])) && (bounds[26] ==
              bounds[37])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[26]) == bounds[26]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[37] - bounds[26]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[37] - bounds[26]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[26] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[37] - bounds[26]) + 0.5);
    xMax = bounds[26] + xMin;
    yMin = xMax - bounds[37];
    yMax = bounds[26];
    u1 = bounds[37];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[37];
    } else if (yMin > 0.0) {
      xMax = bounds[26] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[26];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[26] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[26] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[26] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(e_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = e_reshapes[0].f1->size[0] * e_reshapes[0].f1->size[1];
    e_reshapes[0].f1->size[0] = nx;
    e_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(e_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      e_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = e_reshapes[1].f1->size[0] * e_reshapes[1].f1->size[1];
    e_reshapes[1].f1->size[0] = nx;
    e_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(e_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      e_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = e_reshapes[0].f1->size[0];
    tempCol->size[1] = e_reshapes[0].f1->size[1] + e_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = e_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = e_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = e_reshapes[0].f1->data[i2 +
          e_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = e_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = e_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + e_reshapes[0].f1->size[1])] =
          e_reshapes[1].f1->data[i2 + e_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f5.sw, C->f5.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f5.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f5.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(e_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[4].f1->size[0] * Icell[4].f1->size[1] * Icell[4].f1->size[2];
  Icell[4].f1->size[0] = Iloop->size[0];
  Icell[4].f1->size[1] = Iloop->size[1];
  Icell[4].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[4].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[4].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[5]) || rtIsNaN(bounds[16])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[16] < bounds[5]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[5]) || rtIsInf(bounds[16])) && (bounds[5] ==
              bounds[16])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[5]) == bounds[5]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[16] - bounds[5]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[16] - bounds[5]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[5] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[16] - bounds[5]) + 0.5);
    xMax = bounds[5] + xMin;
    yMin = xMax - bounds[16];
    yMax = bounds[5];
    u1 = bounds[16];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[16];
    } else if (yMin > 0.0) {
      xMax = bounds[5] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[5];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[5] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[5] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[5] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[27]) || rtIsNaN(bounds[38])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[38] < bounds[27]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[27]) || rtIsInf(bounds[38])) && (bounds[27] ==
              bounds[38])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[27]) == bounds[27]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[38] - bounds[27]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[38] - bounds[27]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[27] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[38] - bounds[27]) + 0.5);
    xMax = bounds[27] + xMin;
    yMin = xMax - bounds[38];
    yMax = bounds[27];
    u1 = bounds[38];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[38];
    } else if (yMin > 0.0) {
      xMax = bounds[27] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[27];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[27] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[27] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[27] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(f_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = f_reshapes[0].f1->size[0] * f_reshapes[0].f1->size[1];
    f_reshapes[0].f1->size[0] = nx;
    f_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(f_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      f_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = f_reshapes[1].f1->size[0] * f_reshapes[1].f1->size[1];
    f_reshapes[1].f1->size[0] = nx;
    f_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(f_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      f_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = f_reshapes[0].f1->size[0];
    tempCol->size[1] = f_reshapes[0].f1->size[1] + f_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = f_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = f_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = f_reshapes[0].f1->data[i2 +
          f_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = f_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = f_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + f_reshapes[0].f1->size[1])] =
          f_reshapes[1].f1->data[i2 + f_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f6.sw, C->f6.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f6.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f6.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(f_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[5].f1->size[0] * Icell[5].f1->size[1] * Icell[5].f1->size[2];
  Icell[5].f1->size[0] = Iloop->size[0];
  Icell[5].f1->size[1] = Iloop->size[1];
  Icell[5].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[5].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[5].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[6]) || rtIsNaN(bounds[17])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[17] < bounds[6]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[6]) || rtIsInf(bounds[17])) && (bounds[6] ==
              bounds[17])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[6]) == bounds[6]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[17] - bounds[6]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[17] - bounds[6]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[6] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[17] - bounds[6]) + 0.5);
    xMax = bounds[6] + xMin;
    yMin = xMax - bounds[17];
    yMax = bounds[6];
    u1 = bounds[17];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[17];
    } else if (yMin > 0.0) {
      xMax = bounds[6] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[6];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[6] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[6] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[6] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[28]) || rtIsNaN(bounds[39])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[39] < bounds[28]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[28]) || rtIsInf(bounds[39])) && (bounds[28] ==
              bounds[39])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[28]) == bounds[28]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[39] - bounds[28]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[39] - bounds[28]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[28] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[39] - bounds[28]) + 0.5);
    xMax = bounds[28] + xMin;
    yMin = xMax - bounds[39];
    yMax = bounds[28];
    u1 = bounds[39];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[39];
    } else if (yMin > 0.0) {
      xMax = bounds[28] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[28];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[28] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[28] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[28] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  if (C->f7.isAllEncomp) {
    if (rtIsNaN(sz[0])) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      xIdxs->data[0] = rtNaN;
    } else if (sz[0] < 1.0) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 0;
      emxEnsureCapacity_real_T1(xIdxs, i0);
    } else if (rtIsInf(sz[0]) && (1.0 == sz[0])) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      xIdxs->data[0] = rtNaN;
    } else {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = (int)std::floor(sz[0] - 1.0) + 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      idx = (int)std::floor(sz[0] - 1.0);
      for (i0 = 0; i0 <= idx; i0++) {
        xIdxs->data[xIdxs->size[0] * i0] = 1.0 + (double)i0;
      }
    }

    if (rtIsNaN(sz[1])) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      yIdxs->data[0] = rtNaN;
    } else if (sz[1] < 1.0) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 0;
      emxEnsureCapacity_real_T1(yIdxs, i0);
    } else if (rtIsInf(sz[1]) && (1.0 == sz[1])) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      yIdxs->data[0] = rtNaN;
    } else {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = (int)std::floor(sz[1] - 1.0) + 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      idx = (int)std::floor(sz[1] - 1.0);
      for (i0 = 0; i0 <= idx; i0++) {
        yIdxs->data[yIdxs->size[0] * i0] = 1.0 + (double)i0;
      }
    }

    /* numWorkers = 10; */
  }

  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(g_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = g_reshapes[0].f1->size[0] * g_reshapes[0].f1->size[1];
    g_reshapes[0].f1->size[0] = nx;
    g_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(g_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      g_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = g_reshapes[1].f1->size[0] * g_reshapes[1].f1->size[1];
    g_reshapes[1].f1->size[0] = nx;
    g_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(g_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      g_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = g_reshapes[0].f1->size[0];
    tempCol->size[1] = g_reshapes[0].f1->size[1] + g_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = g_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = g_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = g_reshapes[0].f1->data[i2 +
          g_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = g_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = g_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + g_reshapes[0].f1->size[1])] =
          g_reshapes[1].f1->data[i2 + g_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      checkIntersect(C->f7.sw, C->f7.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f7.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f7.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(g_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[6].f1->size[0] * Icell[6].f1->size[1] * Icell[6].f1->size[2];
  Icell[6].f1->size[0] = Iloop->size[0];
  Icell[6].f1->size[1] = Iloop->size[1];
  Icell[6].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[6].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[6].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[7]) || rtIsNaN(bounds[18])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[18] < bounds[7]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[7]) || rtIsInf(bounds[18])) && (bounds[7] ==
              bounds[18])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[7]) == bounds[7]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[18] - bounds[7]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[18] - bounds[7]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[7] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[18] - bounds[7]) + 0.5);
    xMax = bounds[7] + xMin;
    yMin = xMax - bounds[18];
    yMax = bounds[7];
    u1 = bounds[18];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[18];
    } else if (yMin > 0.0) {
      xMax = bounds[7] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[7];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[7] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[7] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[7] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[29]) || rtIsNaN(bounds[40])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[40] < bounds[29]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[29]) || rtIsInf(bounds[40])) && (bounds[29] ==
              bounds[40])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[29]) == bounds[29]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[40] - bounds[29]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[40] - bounds[29]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[29] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[40] - bounds[29]) + 0.5);
    xMax = bounds[29] + xMin;
    yMin = xMax - bounds[40];
    yMax = bounds[29];
    u1 = bounds[40];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[40];
    } else if (yMin > 0.0) {
      xMax = bounds[29] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[29];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[29] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[29] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[29] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(h_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = h_reshapes[0].f1->size[0] * h_reshapes[0].f1->size[1];
    h_reshapes[0].f1->size[0] = nx;
    h_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(h_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      h_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = h_reshapes[1].f1->size[0] * h_reshapes[1].f1->size[1];
    h_reshapes[1].f1->size[0] = nx;
    h_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(h_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      h_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = h_reshapes[0].f1->size[0];
    tempCol->size[1] = h_reshapes[0].f1->size[1] + h_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = h_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = h_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = h_reshapes[0].f1->data[i2 +
          h_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = h_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = h_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + h_reshapes[0].f1->size[1])] =
          h_reshapes[1].f1->data[i2 + h_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f8.sw, C->f8.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f8.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f8.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(h_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[7].f1->size[0] * Icell[7].f1->size[1] * Icell[7].f1->size[2];
  Icell[7].f1->size[0] = Iloop->size[0];
  Icell[7].f1->size[1] = Iloop->size[1];
  Icell[7].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[7].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[7].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[8]) || rtIsNaN(bounds[19])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[19] < bounds[8]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[8]) || rtIsInf(bounds[19])) && (bounds[8] ==
              bounds[19])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[8]) == bounds[8]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[19] - bounds[8]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[19] - bounds[8]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[8] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[19] - bounds[8]) + 0.5);
    xMax = bounds[8] + xMin;
    yMin = xMax - bounds[19];
    yMax = bounds[8];
    u1 = bounds[19];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[19];
    } else if (yMin > 0.0) {
      xMax = bounds[8] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[8];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[8] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[8] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[8] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[30]) || rtIsNaN(bounds[41])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[41] < bounds[30]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[30]) || rtIsInf(bounds[41])) && (bounds[30] ==
              bounds[41])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[30]) == bounds[30]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[41] - bounds[30]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[41] - bounds[30]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[30] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[41] - bounds[30]) + 0.5);
    xMax = bounds[30] + xMin;
    yMin = xMax - bounds[41];
    yMax = bounds[30];
    u1 = bounds[41];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[41];
    } else if (yMin > 0.0) {
      xMax = bounds[30] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[30];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[30] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[30] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[30] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(i_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = i_reshapes[0].f1->size[0] * i_reshapes[0].f1->size[1];
    i_reshapes[0].f1->size[0] = nx;
    i_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(i_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      i_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = i_reshapes[1].f1->size[0] * i_reshapes[1].f1->size[1];
    i_reshapes[1].f1->size[0] = nx;
    i_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(i_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      i_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = i_reshapes[0].f1->size[0];
    tempCol->size[1] = i_reshapes[0].f1->size[1] + i_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = i_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = i_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = i_reshapes[0].f1->data[i2 +
          i_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = i_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = i_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + i_reshapes[0].f1->size[1])] =
          i_reshapes[1].f1->data[i2 + i_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f9.sw, C->f9.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f9.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f9.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(i_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[8].f1->size[0] * Icell[8].f1->size[1] * Icell[8].f1->size[2];
  Icell[8].f1->size[0] = Iloop->size[0];
  Icell[8].f1->size[1] = Iloop->size[1];
  Icell[8].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[8].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[8].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  if (rtIsNaN(bounds[9]) || rtIsNaN(bounds[20])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[20] < bounds[9]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[9]) || rtIsInf(bounds[20])) && (bounds[9] ==
              bounds[20])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[9]) == bounds[9]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[20] - bounds[9]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[20] - bounds[9]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[9] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[20] - bounds[9]) + 0.5);
    xMax = bounds[9] + xMin;
    yMin = xMax - bounds[20];
    yMax = bounds[9];
    u1 = bounds[20];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[20];
    } else if (yMin > 0.0) {
      xMax = bounds[9] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[9];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[9] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[9] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[9] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[31]) || rtIsNaN(bounds[42])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[42] < bounds[31]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[31]) || rtIsInf(bounds[42])) && (bounds[31] ==
              bounds[42])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[31]) == bounds[31]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[42] - bounds[31]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[42] - bounds[31]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[31] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[42] - bounds[31]) + 0.5);
    xMax = bounds[31] + xMin;
    yMin = xMax - bounds[42];
    yMax = bounds[31];
    u1 = bounds[42];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[42];
    } else if (yMin > 0.0) {
      xMax = bounds[31] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[31];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[31] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[31] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[31] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(j_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = j_reshapes[0].f1->size[0] * j_reshapes[0].f1->size[1];
    j_reshapes[0].f1->size[0] = nx;
    j_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(j_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      j_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = j_reshapes[1].f1->size[0] * j_reshapes[1].f1->size[1];
    j_reshapes[1].f1->size[0] = nx;
    j_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(j_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      j_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = j_reshapes[0].f1->size[0];
    tempCol->size[1] = j_reshapes[0].f1->size[1] + j_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = j_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = j_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = j_reshapes[0].f1->data[i2 +
          j_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = j_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = j_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + j_reshapes[0].f1->size[1])] =
          j_reshapes[1].f1->data[i2 + j_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f10.sw, C->f10.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f10.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f10.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFreeMatrix_cell_wrap_2(j_reshapes);

  /* add this cube's image to the cell */
  i0 = Icell[9].f1->size[0] * Icell[9].f1->size[1] * Icell[9].f1->size[2];
  Icell[9].f1->size[0] = Iloop->size[0];
  Icell[9].f1->size[1] = Iloop->size[1];
  Icell[9].f1->size[2] = 4;
  emxEnsureCapacity_real_T(Icell[9].f1, i0);
  idx = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Icell[9].f1->data[i0] = Iloop->data[i0];
  }

  /* initialize this cube's image (RGB+D) */
  i0 = Iloop->size[0] * Iloop->size[1] * Iloop->size[2];
  Iloop->size[0] = Itemplate->size[0];
  Iloop->size[1] = Itemplate->size[1];
  Iloop->size[2] = 4;
  emxEnsureCapacity_real_T(Iloop, i0);
  idx = Itemplate->size[0] * Itemplate->size[1] * Itemplate->size[2];
  for (i0 = 0; i0 < idx; i0++) {
    Iloop->data[i0] = Itemplate->data[i0];
  }

  emxFree_int8_T(&Itemplate);
  if (rtIsNaN(bounds[10]) || rtIsNaN(bounds[21])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[21] < bounds[10]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[10]) || rtIsInf(bounds[21])) && (bounds[10] ==
              bounds[21])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[10]) == bounds[10]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[21] - bounds[10]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[21] - bounds[10]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[10] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[21] - bounds[10]) + 0.5);
    xMax = bounds[10] + xMin;
    yMin = xMax - bounds[21];
    yMax = bounds[10];
    u1 = bounds[21];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[21];
    } else if (yMin > 0.0) {
      xMax = bounds[10] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[10];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[10] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[10] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[10] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[32]) || rtIsNaN(bounds[43])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[43] < bounds[32]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[32]) || rtIsInf(bounds[43])) && (bounds[32] ==
              bounds[43])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[32]) == bounds[32]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[43] - bounds[32]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[43] - bounds[32]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[32] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[43] - bounds[32]) + 0.5);
    xMax = bounds[32] + xMin;
    yMin = xMax - bounds[43];
    yMax = bounds[32];
    u1 = bounds[43];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[43];
    } else if (yMin > 0.0) {
      xMax = bounds[32] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[32];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[32] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[32] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[32] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* do a parfor or regular for loop */
  /*      numWorkers = 10; */
  /* check to see if this is actually the all encompassing box */
  /*      parfor (ii = xIdxs, numWorkers) */
  i0 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
  ii = 0;
  emxInitMatrix_cell_wrap_2(k_reshapes);
  while (ii <= i0 - 1) {
    xMin = xIdxs->data[0] + (double)ii;

    /* temporary row */
    nx = yIdxs->size[1];
    if (!(nx == 0)) {
      nx = yIdxs->size[1];
    } else if (!(yIdxs->size[1] == 0)) {
      nx = yIdxs->size[1];
    } else {
      nx = yIdxs->size[1];
      if (nx > 0) {
        nx = yIdxs->size[1];
      } else {
        nx = 0;
      }

      if (yIdxs->size[1] > nx) {
        nx = yIdxs->size[1];
      }
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes) {
      nm1d2 = 3;
    } else {
      nm1d2 = yIdxs->size[1];
      if (!(nm1d2 == 0)) {
        nm1d2 = 3;
      } else {
        nm1d2 = 0;
      }
    }

    i1 = k_reshapes[0].f1->size[0] * k_reshapes[0].f1->size[1];
    k_reshapes[0].f1->size[0] = nx;
    k_reshapes[0].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(k_reshapes[0].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      k_reshapes[0].f1->data[i1] = 1.0;
    }

    if (empty_non_axis_sizes || (!(yIdxs->size[1] == 0))) {
      nm1d2 = 1;
    } else {
      nm1d2 = 0;
    }

    i1 = k_reshapes[1].f1->size[0] * k_reshapes[1].f1->size[1];
    k_reshapes[1].f1->size[0] = nx;
    k_reshapes[1].f1->size[1] = nm1d2;
    emxEnsureCapacity_real_T1(k_reshapes[1].f1, i1);
    idx = nx * nm1d2;
    for (i1 = 0; i1 < idx; i1++) {
      k_reshapes[1].f1->data[i1] = 0.0;
    }

    i1 = tempCol->size[0] * tempCol->size[1];
    tempCol->size[0] = k_reshapes[0].f1->size[0];
    tempCol->size[1] = k_reshapes[0].f1->size[1] + k_reshapes[1].f1->size[1];
    emxEnsureCapacity_real_T1(tempCol, i1);
    idx = k_reshapes[0].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = k_reshapes[0].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * i1] = k_reshapes[0].f1->data[i2 +
          k_reshapes[0].f1->size[0] * i1];
      }
    }

    idx = k_reshapes[1].f1->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      nm1d2 = k_reshapes[1].f1->size[0];
      for (i2 = 0; i2 < nm1d2; i2++) {
        tempCol->data[i2 + tempCol->size[0] * (i1 + k_reshapes[0].f1->size[1])] =
          k_reshapes[1].f1->data[i2 + k_reshapes[1].f1->size[0] * i1];
      }
    }

    i1 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
    for (jj = 0; jj < i1; jj++) {
      yMax = yIdxs->data[0] + (double)jj;

      /* get pixel vector and then rotate it into the inertial frame */
      /*   QUATROTATE Rotate a vector by a quaternion (copied for use in MATLAB Coder). */
      /*    N = QUATROTATE( Q, R ) calculates the rotated vector, N, for a */
      /*    quaternion, Q, and a vector, R.  Q is either a M-by-4 matrix */
      /*    containing M quaternions or a single 1-by4 quaternion.  R */
      /*    is either a M-by-3 matrix or a single 1-by-3 vector.  N returns an */
      /*    M-by-3 matrix of rotated vectors.  Each element of Q and R must be a */
      /*    real number.  Additionally, Q has its scalar number as the first column. */
      /*  */
      /*    Examples: */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
      /*       r = [1 1 1; 2 3 4]; */
      /*       n = quatrotate( q, r )  */
      /*  */
      /*    See also QUAT2DCM, QUATCONJ, QUATDIVIDE, QUATINV, QUATMOD, */
      /*    QUATMULTIPLY, QUATNORM, QUATNORMALIZE. */
      /*    Copyright 2000-2010 The MathWorks, Inc. */
      /* get M */
      /*  Q is 1-by-4 */
      /*  check intersect */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = q[i2];
      }

      quatconj(Verts);
      quat2dcmCoder(Verts, dv0);
      for (i2 = 0; i2 < 3; i2++) {
        dv1[i2] = 0.0;
        for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
          dv1[i2] += dv0[i2 + 3 * nm1d2] * V[(((int)yMax + 480 * ((int)xMin - 1))
            + 307200 * nm1d2) - 1];
        }
      }

      b_checkIntersect(C->f11.sw, C->f11.faces, P, dv1, &xMax, &yMin);

      /* intitialize pixel */
      for (i2 = 0; i2 < 4; i2++) {
        Verts[i2] = 0.0;
      }

      /* assign distance */
      Verts[3] = yMin;
      if (!(xMax == 0.0)) {
        if (xMax == 7.0) {
          /* we hit an edge */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f11.ec[i2];
          }
        } else {
          /* else we hit an face and need to get that edges color */
          for (i2 = 0; i2 < 3; i2++) {
            Verts[i2] = C->f11.faces[(int)xMax - 1].color[i2];
          }
        }

        /* build temporary row */
        i2 = x->size[0] * x->size[1];
        x->size[0] = 1;
        x->size[1] = yIdxs->size[1];
        emxEnsureCapacity_boolean_T(x, i2);
        idx = yIdxs->size[0] * yIdxs->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          x->data[i2] = (yIdxs->data[i2] == yMax);
        }

        nx = x->size[1];
        idx = 0;
        i2 = b_ii->size[0] * b_ii->size[1];
        b_ii->size[0] = 1;
        b_ii->size[1] = x->size[1];
        emxEnsureCapacity_int32_T1(b_ii, i2);
        nm1d2 = 1;
        exitg1 = false;
        while ((!exitg1) && (nm1d2 <= nx)) {
          if (x->data[nm1d2 - 1]) {
            idx++;
            b_ii->data[idx - 1] = nm1d2;
            if (idx >= nx) {
              exitg1 = true;
            } else {
              nm1d2++;
            }
          } else {
            nm1d2++;
          }
        }

        if (x->size[1] == 1) {
          if (idx == 0) {
            i2 = b_ii->size[0] * b_ii->size[1];
            b_ii->size[0] = 1;
            b_ii->size[1] = 0;
            emxEnsureCapacity_int32_T1(b_ii, i2);
          }
        } else {
          i2 = b_ii->size[0] * b_ii->size[1];
          if (1 > idx) {
            b_ii->size[1] = 0;
          } else {
            b_ii->size[1] = idx;
          }

          emxEnsureCapacity_int32_T1(b_ii, i2);
        }

        i2 = targRow->size[0] * targRow->size[1];
        targRow->size[0] = 1;
        targRow->size[1] = b_ii->size[1];
        emxEnsureCapacity_int32_T1(targRow, i2);
        idx = b_ii->size[0] * b_ii->size[1];
        for (i2 = 0; i2 < idx; i2++) {
          targRow->data[i2] = b_ii->data[i2];
        }

        nm1d2 = targRow->data[0];
        for (i2 = 0; i2 < 4; i2++) {
          tempCol->data[(nm1d2 + tempCol->size[0] * i2) - 1] = Verts[i2];
        }
      } else {
        /* we hit nothing */
      }
    }

    i1 = r0->size[0];
    r0->size[0] = yIdxs->size[1];
    emxEnsureCapacity_int32_T(r0, i1);
    idx = yIdxs->size[1];
    for (i1 = 0; i1 < idx; i1++) {
      r0->data[i1] = (int)yIdxs->data[yIdxs->size[0] * i1] - 1;
    }

    nm1d2 = r0->size[0];
    for (i1 = 0; i1 < 4; i1++) {
      for (i2 = 0; i2 < nm1d2; i2++) {
        Iloop->data[(r0->data[i2] + Iloop->size[0] * ((int)xMin - 1)) +
          Iloop->size[0] * Iloop->size[1] * i1] = tempCol->data[i2 + nm1d2 * i1];
      }
    }

    ii++;
  }

  emxFree_int32_T(&b_ii);
  emxFree_boolean_T(&x);
  emxFreeMatrix_cell_wrap_2(k_reshapes);
  emxFree_int32_T(&r0);
  emxFree_int32_T(&targRow);
  emxFree_real_T(&tempCol);

  /* add this cube's image to the cell */
  /* create final output */
  i0 = I->size[0] * I->size[1] * I->size[2];
  I->size[0] = (int)sz[1];
  I->size[1] = (int)sz[0];
  I->size[2] = 4;
  emxEnsureCapacity_real_T(I, i0);
  idx = ((int)sz[1] * (int)sz[0]) << 2;
  for (i0 = 0; i0 < idx; i0++) {
    I->data[i0] = 1.0;
  }

  idx = (int)sz[0];
  for (i0 = 0; i0 < idx; i0++) {
    nm1d2 = (int)sz[1];
    for (i1 = 0; i1 < nm1d2; i1++) {
      I->data[(i1 + I->size[0] * i0) + I->size[0] * I->size[1] * 3] = 0.0;
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[0]) || rtIsNaN(bounds[11])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[11] < bounds[0]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[0]) || rtIsInf(bounds[11])) && (bounds[0] ==
              bounds[11])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[0]) == bounds[0]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[11] - bounds[0]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[11] - bounds[0]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[0] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[11] - bounds[0]) + 0.5);
    xMax = bounds[0] + xMin;
    yMin = xMax - bounds[11];
    yMax = bounds[0];
    u1 = bounds[11];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[11];
    } else if (yMin > 0.0) {
      xMax = bounds[0] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[0];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[0] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[0] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[0] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[22]) || rtIsNaN(bounds[33])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[33] < bounds[22]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[22]) || rtIsInf(bounds[33])) && (bounds[22] ==
              bounds[33])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[22]) == bounds[22]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[33] - bounds[22]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[33] - bounds[22]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[22] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[33] - bounds[22]) + 0.5);
    xMax = bounds[22] + xMin;
    yMin = xMax - bounds[33];
    yMax = bounds[22];
    u1 = bounds[33];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[33];
    } else if (yMin > 0.0) {
      xMax = bounds[22] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[22];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[22] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[22] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[22] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[0].f1->data[(((int)yMax + Icell[0].f1->size[0] * ((int)xMin -
              1)) + Icell[0].f1->size[0] * Icell[0].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[0].f1->data[(((int)yMax +
              Icell[0].f1->size[0] * ((int)xMin - 1)) + Icell[0].f1->size[0] *
              Icell[0].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[0].f1->data[(((int)yMax +
                Icell[0].f1->size[0] * ((int)xMin - 1)) + Icell[0].f1->size[0] *
               Icell[0].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[0].f1->data[(((int)yMax +
                Icell[0].f1->size[0] * ((int)xMin - 1)) + Icell[0].f1->size[0] *
                Icell[0].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[1]) || rtIsNaN(bounds[12])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[12] < bounds[1]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[1]) || rtIsInf(bounds[12])) && (bounds[1] ==
              bounds[12])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[1]) == bounds[1]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[12] - bounds[1]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[12] - bounds[1]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[1] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[12] - bounds[1]) + 0.5);
    xMax = bounds[1] + xMin;
    yMin = xMax - bounds[12];
    yMax = bounds[1];
    u1 = bounds[12];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[12];
    } else if (yMin > 0.0) {
      xMax = bounds[1] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[1];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[1] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[1] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[1] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[23]) || rtIsNaN(bounds[34])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[34] < bounds[23]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[23]) || rtIsInf(bounds[34])) && (bounds[23] ==
              bounds[34])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[23]) == bounds[23]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[34] - bounds[23]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[34] - bounds[23]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[23] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[34] - bounds[23]) + 0.5);
    xMax = bounds[23] + xMin;
    yMin = xMax - bounds[34];
    yMax = bounds[23];
    u1 = bounds[34];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[34];
    } else if (yMin > 0.0) {
      xMax = bounds[23] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[23];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[23] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[23] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[23] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[1].f1->data[(((int)yMax + Icell[1].f1->size[0] * ((int)xMin -
              1)) + Icell[1].f1->size[0] * Icell[1].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[1].f1->data[(((int)yMax +
              Icell[1].f1->size[0] * ((int)xMin - 1)) + Icell[1].f1->size[0] *
              Icell[1].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[1].f1->data[(((int)yMax +
                Icell[1].f1->size[0] * ((int)xMin - 1)) + Icell[1].f1->size[0] *
               Icell[1].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[1].f1->data[(((int)yMax +
                Icell[1].f1->size[0] * ((int)xMin - 1)) + Icell[1].f1->size[0] *
                Icell[1].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[2]) || rtIsNaN(bounds[13])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[13] < bounds[2]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[2]) || rtIsInf(bounds[13])) && (bounds[2] ==
              bounds[13])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[2]) == bounds[2]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[13] - bounds[2]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[13] - bounds[2]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[2] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[13] - bounds[2]) + 0.5);
    xMax = bounds[2] + xMin;
    yMin = xMax - bounds[13];
    yMax = bounds[2];
    u1 = bounds[13];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[13];
    } else if (yMin > 0.0) {
      xMax = bounds[2] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[2];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[2] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[2] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[2] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[24]) || rtIsNaN(bounds[35])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[35] < bounds[24]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[24]) || rtIsInf(bounds[35])) && (bounds[24] ==
              bounds[35])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[24]) == bounds[24]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[35] - bounds[24]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[35] - bounds[24]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[24] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[35] - bounds[24]) + 0.5);
    xMax = bounds[24] + xMin;
    yMin = xMax - bounds[35];
    yMax = bounds[24];
    u1 = bounds[35];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[35];
    } else if (yMin > 0.0) {
      xMax = bounds[24] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[24];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[24] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[24] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[24] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[2].f1->data[(((int)yMax + Icell[2].f1->size[0] * ((int)xMin -
              1)) + Icell[2].f1->size[0] * Icell[2].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[2].f1->data[(((int)yMax +
              Icell[2].f1->size[0] * ((int)xMin - 1)) + Icell[2].f1->size[0] *
              Icell[2].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[2].f1->data[(((int)yMax +
                Icell[2].f1->size[0] * ((int)xMin - 1)) + Icell[2].f1->size[0] *
               Icell[2].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[2].f1->data[(((int)yMax +
                Icell[2].f1->size[0] * ((int)xMin - 1)) + Icell[2].f1->size[0] *
                Icell[2].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[3]) || rtIsNaN(bounds[14])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[14] < bounds[3]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[3]) || rtIsInf(bounds[14])) && (bounds[3] ==
              bounds[14])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[3]) == bounds[3]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[14] - bounds[3]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[14] - bounds[3]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[3] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[14] - bounds[3]) + 0.5);
    xMax = bounds[3] + xMin;
    yMin = xMax - bounds[14];
    yMax = bounds[3];
    u1 = bounds[14];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[14];
    } else if (yMin > 0.0) {
      xMax = bounds[3] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[3];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[3] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[3] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[3] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[25]) || rtIsNaN(bounds[36])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[36] < bounds[25]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[25]) || rtIsInf(bounds[36])) && (bounds[25] ==
              bounds[36])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[25]) == bounds[25]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[36] - bounds[25]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[36] - bounds[25]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[25] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[36] - bounds[25]) + 0.5);
    xMax = bounds[25] + xMin;
    yMin = xMax - bounds[36];
    yMax = bounds[25];
    u1 = bounds[36];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[36];
    } else if (yMin > 0.0) {
      xMax = bounds[25] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[25];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[25] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[25] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[25] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[3].f1->data[(((int)yMax + Icell[3].f1->size[0] * ((int)xMin -
              1)) + Icell[3].f1->size[0] * Icell[3].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[3].f1->data[(((int)yMax +
              Icell[3].f1->size[0] * ((int)xMin - 1)) + Icell[3].f1->size[0] *
              Icell[3].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[3].f1->data[(((int)yMax +
                Icell[3].f1->size[0] * ((int)xMin - 1)) + Icell[3].f1->size[0] *
               Icell[3].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[3].f1->data[(((int)yMax +
                Icell[3].f1->size[0] * ((int)xMin - 1)) + Icell[3].f1->size[0] *
                Icell[3].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[4]) || rtIsNaN(bounds[15])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[15] < bounds[4]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[4]) || rtIsInf(bounds[15])) && (bounds[4] ==
              bounds[15])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[4]) == bounds[4]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[15] - bounds[4]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[15] - bounds[4]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[4] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[15] - bounds[4]) + 0.5);
    xMax = bounds[4] + xMin;
    yMin = xMax - bounds[15];
    yMax = bounds[4];
    u1 = bounds[15];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[15];
    } else if (yMin > 0.0) {
      xMax = bounds[4] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[4];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[4] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[4] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[4] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[26]) || rtIsNaN(bounds[37])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[37] < bounds[26]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[26]) || rtIsInf(bounds[37])) && (bounds[26] ==
              bounds[37])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[26]) == bounds[26]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[37] - bounds[26]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[37] - bounds[26]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[26] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[37] - bounds[26]) + 0.5);
    xMax = bounds[26] + xMin;
    yMin = xMax - bounds[37];
    yMax = bounds[26];
    u1 = bounds[37];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[37];
    } else if (yMin > 0.0) {
      xMax = bounds[26] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[26];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[26] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[26] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[26] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[4].f1->data[(((int)yMax + Icell[4].f1->size[0] * ((int)xMin -
              1)) + Icell[4].f1->size[0] * Icell[4].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[4].f1->data[(((int)yMax +
              Icell[4].f1->size[0] * ((int)xMin - 1)) + Icell[4].f1->size[0] *
              Icell[4].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[4].f1->data[(((int)yMax +
                Icell[4].f1->size[0] * ((int)xMin - 1)) + Icell[4].f1->size[0] *
               Icell[4].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[4].f1->data[(((int)yMax +
                Icell[4].f1->size[0] * ((int)xMin - 1)) + Icell[4].f1->size[0] *
                Icell[4].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[5]) || rtIsNaN(bounds[16])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[16] < bounds[5]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[5]) || rtIsInf(bounds[16])) && (bounds[5] ==
              bounds[16])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[5]) == bounds[5]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[16] - bounds[5]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[16] - bounds[5]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[5] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[16] - bounds[5]) + 0.5);
    xMax = bounds[5] + xMin;
    yMin = xMax - bounds[16];
    yMax = bounds[5];
    u1 = bounds[16];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[16];
    } else if (yMin > 0.0) {
      xMax = bounds[5] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[5];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[5] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[5] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[5] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[27]) || rtIsNaN(bounds[38])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[38] < bounds[27]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[27]) || rtIsInf(bounds[38])) && (bounds[27] ==
              bounds[38])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[27]) == bounds[27]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[38] - bounds[27]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[38] - bounds[27]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[27] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[38] - bounds[27]) + 0.5);
    xMax = bounds[27] + xMin;
    yMin = xMax - bounds[38];
    yMax = bounds[27];
    u1 = bounds[38];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[38];
    } else if (yMin > 0.0) {
      xMax = bounds[27] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[27];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[27] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[27] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[27] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[5].f1->data[(((int)yMax + Icell[5].f1->size[0] * ((int)xMin -
              1)) + Icell[5].f1->size[0] * Icell[5].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[5].f1->data[(((int)yMax +
              Icell[5].f1->size[0] * ((int)xMin - 1)) + Icell[5].f1->size[0] *
              Icell[5].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[5].f1->data[(((int)yMax +
                Icell[5].f1->size[0] * ((int)xMin - 1)) + Icell[5].f1->size[0] *
               Icell[5].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[5].f1->data[(((int)yMax +
                Icell[5].f1->size[0] * ((int)xMin - 1)) + Icell[5].f1->size[0] *
                Icell[5].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[6]) || rtIsNaN(bounds[17])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[17] < bounds[6]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[6]) || rtIsInf(bounds[17])) && (bounds[6] ==
              bounds[17])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[6]) == bounds[6]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[17] - bounds[6]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[17] - bounds[6]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[6] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[17] - bounds[6]) + 0.5);
    xMax = bounds[6] + xMin;
    yMin = xMax - bounds[17];
    yMax = bounds[6];
    u1 = bounds[17];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[17];
    } else if (yMin > 0.0) {
      xMax = bounds[6] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[6];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[6] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[6] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[6] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[28]) || rtIsNaN(bounds[39])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[39] < bounds[28]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[28]) || rtIsInf(bounds[39])) && (bounds[28] ==
              bounds[39])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[28]) == bounds[28]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[39] - bounds[28]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[39] - bounds[28]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[28] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[39] - bounds[28]) + 0.5);
    xMax = bounds[28] + xMin;
    yMin = xMax - bounds[39];
    yMax = bounds[28];
    u1 = bounds[39];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[39];
    } else if (yMin > 0.0) {
      xMax = bounds[28] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[28];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[28] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[28] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[28] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  if (C->f7.isAllEncomp) {
    if (rtIsNaN(sz[0])) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      xIdxs->data[0] = rtNaN;
    } else if (sz[0] < 1.0) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 0;
      emxEnsureCapacity_real_T1(xIdxs, i0);
    } else if (rtIsInf(sz[0]) && (1.0 == sz[0])) {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      xIdxs->data[0] = rtNaN;
    } else {
      i0 = xIdxs->size[0] * xIdxs->size[1];
      xIdxs->size[0] = 1;
      xIdxs->size[1] = (int)std::floor(sz[0] - 1.0) + 1;
      emxEnsureCapacity_real_T1(xIdxs, i0);
      idx = (int)std::floor(sz[0] - 1.0);
      for (i0 = 0; i0 <= idx; i0++) {
        xIdxs->data[xIdxs->size[0] * i0] = 1.0 + (double)i0;
      }
    }

    if (rtIsNaN(sz[1])) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      yIdxs->data[0] = rtNaN;
    } else if (sz[1] < 1.0) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 0;
      emxEnsureCapacity_real_T1(yIdxs, i0);
    } else if (rtIsInf(sz[1]) && (1.0 == sz[1])) {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      yIdxs->data[0] = rtNaN;
    } else {
      i0 = yIdxs->size[0] * yIdxs->size[1];
      yIdxs->size[0] = 1;
      yIdxs->size[1] = (int)std::floor(sz[1] - 1.0) + 1;
      emxEnsureCapacity_real_T1(yIdxs, i0);
      idx = (int)std::floor(sz[1] - 1.0);
      for (i0 = 0; i0 <= idx; i0++) {
        yIdxs->data[yIdxs->size[0] * i0] = 1.0 + (double)i0;
      }
    }
  }

  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[6].f1->data[(((int)yMax + Icell[6].f1->size[0] * ((int)xMin -
              1)) + Icell[6].f1->size[0] * Icell[6].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[6].f1->data[(((int)yMax +
              Icell[6].f1->size[0] * ((int)xMin - 1)) + Icell[6].f1->size[0] *
              Icell[6].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[6].f1->data[(((int)yMax +
                Icell[6].f1->size[0] * ((int)xMin - 1)) + Icell[6].f1->size[0] *
               Icell[6].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[6].f1->data[(((int)yMax +
                Icell[6].f1->size[0] * ((int)xMin - 1)) + Icell[6].f1->size[0] *
                Icell[6].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[7]) || rtIsNaN(bounds[18])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[18] < bounds[7]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[7]) || rtIsInf(bounds[18])) && (bounds[7] ==
              bounds[18])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[7]) == bounds[7]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[18] - bounds[7]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[18] - bounds[7]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[7] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[18] - bounds[7]) + 0.5);
    xMax = bounds[7] + xMin;
    yMin = xMax - bounds[18];
    yMax = bounds[7];
    u1 = bounds[18];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[18];
    } else if (yMin > 0.0) {
      xMax = bounds[7] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[7];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[7] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[7] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[7] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[29]) || rtIsNaN(bounds[40])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[40] < bounds[29]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[29]) || rtIsInf(bounds[40])) && (bounds[29] ==
              bounds[40])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[29]) == bounds[29]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[40] - bounds[29]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[40] - bounds[29]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[29] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[40] - bounds[29]) + 0.5);
    xMax = bounds[29] + xMin;
    yMin = xMax - bounds[40];
    yMax = bounds[29];
    u1 = bounds[40];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[40];
    } else if (yMin > 0.0) {
      xMax = bounds[29] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[29];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[29] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[29] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[29] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[7].f1->data[(((int)yMax + Icell[7].f1->size[0] * ((int)xMin -
              1)) + Icell[7].f1->size[0] * Icell[7].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[7].f1->data[(((int)yMax +
              Icell[7].f1->size[0] * ((int)xMin - 1)) + Icell[7].f1->size[0] *
              Icell[7].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[7].f1->data[(((int)yMax +
                Icell[7].f1->size[0] * ((int)xMin - 1)) + Icell[7].f1->size[0] *
               Icell[7].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[7].f1->data[(((int)yMax +
                Icell[7].f1->size[0] * ((int)xMin - 1)) + Icell[7].f1->size[0] *
                Icell[7].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[8]) || rtIsNaN(bounds[19])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[19] < bounds[8]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[8]) || rtIsInf(bounds[19])) && (bounds[8] ==
              bounds[19])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[8]) == bounds[8]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[19] - bounds[8]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[19] - bounds[8]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[8] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[19] - bounds[8]) + 0.5);
    xMax = bounds[8] + xMin;
    yMin = xMax - bounds[19];
    yMax = bounds[8];
    u1 = bounds[19];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[19];
    } else if (yMin > 0.0) {
      xMax = bounds[8] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[8];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[8] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[8] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[8] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[30]) || rtIsNaN(bounds[41])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[41] < bounds[30]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[30]) || rtIsInf(bounds[41])) && (bounds[30] ==
              bounds[41])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[30]) == bounds[30]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[41] - bounds[30]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[41] - bounds[30]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[30] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[41] - bounds[30]) + 0.5);
    xMax = bounds[30] + xMin;
    yMin = xMax - bounds[41];
    yMax = bounds[30];
    u1 = bounds[41];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[41];
    } else if (yMin > 0.0) {
      xMax = bounds[30] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[30];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[30] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[30] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[30] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[8].f1->data[(((int)yMax + Icell[8].f1->size[0] * ((int)xMin -
              1)) + Icell[8].f1->size[0] * Icell[8].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[8].f1->data[(((int)yMax +
              Icell[8].f1->size[0] * ((int)xMin - 1)) + Icell[8].f1->size[0] *
              Icell[8].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[8].f1->data[(((int)yMax +
                Icell[8].f1->size[0] * ((int)xMin - 1)) + Icell[8].f1->size[0] *
               Icell[8].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[8].f1->data[(((int)yMax +
                Icell[8].f1->size[0] * ((int)xMin - 1)) + Icell[8].f1->size[0] *
                Icell[8].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[9]) || rtIsNaN(bounds[20])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[20] < bounds[9]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[9]) || rtIsInf(bounds[20])) && (bounds[9] ==
              bounds[20])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[9]) == bounds[9]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[20] - bounds[9]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[20] - bounds[9]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[9] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[20] - bounds[9]) + 0.5);
    xMax = bounds[9] + xMin;
    yMin = xMax - bounds[20];
    yMax = bounds[9];
    u1 = bounds[20];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[20];
    } else if (yMin > 0.0) {
      xMax = bounds[9] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[9];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[9] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[9] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[9] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[31]) || rtIsNaN(bounds[42])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[42] < bounds[31]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[31]) || rtIsInf(bounds[42])) && (bounds[31] ==
              bounds[42])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[31]) == bounds[31]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[42] - bounds[31]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[42] - bounds[31]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[31] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[42] - bounds[31]) + 0.5);
    xMax = bounds[31] + xMin;
    yMin = xMax - bounds[42];
    yMax = bounds[31];
    u1 = bounds[42];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[42];
    } else if (yMin > 0.0) {
      xMax = bounds[31] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[31];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[31] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[31] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[31] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Icell[9].f1->data[(((int)yMax + Icell[9].f1->size[0] * ((int)xMin -
              1)) + Icell[9].f1->size[0] * Icell[9].f1->size[1] * 3) - 1] == 0.0))
      {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Icell[9].f1->data[(((int)yMax +
              Icell[9].f1->size[0] * ((int)xMin - 1)) + Icell[9].f1->size[0] *
              Icell[9].f1->size[1] * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Icell[9].f1->data[(((int)yMax +
                Icell[9].f1->size[0] * ((int)xMin - 1)) + Icell[9].f1->size[0] *
               Icell[9].f1->size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Icell[9].f1->data[(((int)yMax +
                Icell[9].f1->size[0] * ((int)xMin - 1)) + Icell[9].f1->size[0] *
                Icell[9].f1->size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  emxFreeMatrix_cell_wrap_1(Icell);

  /* it might speed things up to integrate this loop with the one above */
  if (rtIsNaN(bounds[10]) || rtIsNaN(bounds[21])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (bounds[21] < bounds[10]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(xIdxs, i0);
  } else if ((rtIsInf(bounds[10]) || rtIsInf(bounds[21])) && (bounds[10] ==
              bounds[21])) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    xIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[10]) == bounds[10]) {
    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = (int)std::floor(bounds[21] - bounds[10]) + 1;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    idx = (int)std::floor(bounds[21] - bounds[10]);
    for (i0 = 0; i0 <= idx; i0++) {
      xIdxs->data[xIdxs->size[0] * i0] = bounds[10] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[21] - bounds[10]) + 0.5);
    xMax = bounds[10] + xMin;
    yMin = xMax - bounds[21];
    yMax = bounds[10];
    u1 = bounds[21];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[21];
    } else if (yMin > 0.0) {
      xMax = bounds[10] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = xIdxs->size[0] * xIdxs->size[1];
    xIdxs->size[0] = 1;
    xIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(xIdxs, i0);
    if (idx > 0) {
      xIdxs->data[0] = bounds[10];
      if (idx > 1) {
        xIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          xIdxs->data[nx] = bounds[10] + (double)nx;
          xIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          xIdxs->data[nm1d2] = (bounds[10] + xMax) / 2.0;
        } else {
          xIdxs->data[nm1d2] = bounds[10] + (double)nm1d2;
          xIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  if (rtIsNaN(bounds[32]) || rtIsNaN(bounds[43])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (bounds[43] < bounds[32]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 0;
    emxEnsureCapacity_real_T1(yIdxs, i0);
  } else if ((rtIsInf(bounds[32]) || rtIsInf(bounds[43])) && (bounds[32] ==
              bounds[43])) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    yIdxs->data[0] = rtNaN;
  } else if (std::floor(bounds[32]) == bounds[32]) {
    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = (int)std::floor(bounds[43] - bounds[32]) + 1;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    idx = (int)std::floor(bounds[43] - bounds[32]);
    for (i0 = 0; i0 <= idx; i0++) {
      yIdxs->data[yIdxs->size[0] * i0] = bounds[32] + (double)i0;
    }
  } else {
    xMin = std::floor((bounds[43] - bounds[32]) + 0.5);
    xMax = bounds[32] + xMin;
    yMin = xMax - bounds[43];
    yMax = bounds[32];
    u1 = bounds[43];
    if ((yMax > u1) || rtIsNaN(u1)) {
      u1 = yMax;
    }

    if (std::abs(yMin) < 4.4408920985006262E-16 * u1) {
      xMin++;
      xMax = bounds[43];
    } else if (yMin > 0.0) {
      xMax = bounds[32] + (xMin - 1.0);
    } else {
      xMin++;
    }

    if (xMin >= 0.0) {
      idx = (int)xMin;
    } else {
      idx = 0;
    }

    i0 = yIdxs->size[0] * yIdxs->size[1];
    yIdxs->size[0] = 1;
    yIdxs->size[1] = idx;
    emxEnsureCapacity_real_T1(yIdxs, i0);
    if (idx > 0) {
      yIdxs->data[0] = bounds[32];
      if (idx > 1) {
        yIdxs->data[idx - 1] = xMax;
        nm1d2 = (idx - 1) / 2;
        for (nx = 1; nx < nm1d2; nx++) {
          yIdxs->data[nx] = bounds[32] + (double)nx;
          yIdxs->data[(idx - nx) - 1] = xMax - (double)nx;
        }

        if (nm1d2 << 1 == idx - 1) {
          yIdxs->data[nm1d2] = (bounds[32] + xMax) / 2.0;
        } else {
          yIdxs->data[nm1d2] = bounds[32] + (double)nm1d2;
          yIdxs->data[nm1d2 + 1] = xMax - (double)nm1d2;
        }
      }
    }
  }

  /* check to see if this is actually the all encompassing box */
  i0 = (int)(yIdxs->data[yIdxs->size[1] - 1] + (1.0 - yIdxs->data[0]));
  for (jj = 0; jj < i0; jj++) {
    yMax = yIdxs->data[0] + (double)jj;

    /* note: a parfor loop is slower here */
    i1 = (int)(xIdxs->data[xIdxs->size[1] - 1] + (1.0 - xIdxs->data[0]));
    for (nm1d2 = 0; nm1d2 < i1; nm1d2++) {
      xMin = xIdxs->data[0] + (double)nm1d2;
      if (!(Iloop->data[(((int)yMax + Iloop->size[0] * ((int)xMin - 1)) +
                         Iloop->size[0] * Iloop->size[1] * 3) - 1] == 0.0)) {
        if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * 3) - 1] == 0.0) {
          /* this is the first occupied pixel we've observed */
          for (i2 = 0; i2 < 4; i2++) {
            I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                     I->size[1] * i2) - 1] = Iloop->data[(((int)yMax +
              Iloop->size[0] * ((int)xMin - 1)) + Iloop->size[0] * Iloop->size[1]
              * i2) - 1];
          }
        } else {
          if (I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * 3) - 1] > Iloop->data[(((int)yMax +
                Iloop->size[0] * ((int)xMin - 1)) + Iloop->size[0] * Iloop->
               size[1] * 3) - 1]) {
            /* this pixel is closer than the previously observed one */
            for (i2 = 0; i2 < 4; i2++) {
              I->data[(((int)yMax + I->size[0] * ((int)xMin - 1)) + I->size[0] *
                       I->size[1] * i2) - 1] = Iloop->data[(((int)yMax +
                Iloop->size[0] * ((int)xMin - 1)) + Iloop->size[0] * Iloop->
                size[1] * i2) - 1];
            }
          }
        }
      } else {
        /* there is nothing here */
      }
    }
  }

  emxFree_real_T(&yIdxs);
  emxFree_real_T(&xIdxs);
  emxFree_real_T(&Iloop);
}

/* End of code generation (createImage.cpp) */
