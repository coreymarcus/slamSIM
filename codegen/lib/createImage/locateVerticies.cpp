/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * locateVerticies.cpp
 *
 * Code generation for function 'locateVerticies'
 *
 */

/* Include files */
#include <math.h>
#include "rt_nonfinite.h"
#include "createImage.h"
#include "locateVerticies.h"
#include "quat2dcmCoder.h"

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void b_locateVerticies(const struct1_T C_faces[6], const double P[3], const
  double q[4], const double K[9], double Verts[4])
{
  double dv3[9];
  int ii;
  double pts[48];
  int idx;
  int k;
  boolean_T exitg1;
  double ex;
  double b_P[12];
  int i4;
  double b_C_faces[12];
  int jj;
  double ptsInCam[12];
  double x[3];

  /* LOCATEVERTICIES finds the min and max pixels which will be occupied by the */
  /* cube to speed up image generation */
  /*   */
  /*  Inputs */
  /*  C - cube structure */
  /*  P - [3x1] camera position in world frame */
  /*  q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts */
  /*  K = [3x3] camera calibration matrix */
  /*   */
  /*  Outputs */
  /*  Verts = [pixXmin, pixXmax, pixYmin, pixYmax] */
  /* initialize points */
  /* really a cube has 8 unique verticies, but I'm not sure how to identify */
  /* them so we'll go through all of the potentials */
  quat2dcmCoder(q, dv3);
  for (ii = 0; ii < 6; ii++) {
    /* extract points */
    /* translate to camera */
    /* rotate into camera frame */
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
    for (k = 0; k < 3; k++) {
      b_P[k] = P[k];
      b_P[3 + k] = P[k];
      b_P[6 + k] = P[k];
      b_P[9 + k] = P[k];
    }

    for (k = 0; k < 4; k++) {
      for (i4 = 0; i4 < 3; i4++) {
        b_C_faces[i4 + 3 * k] = C_faces[ii].vertex[i4 + 3 * k] - b_P[i4 + 3 * k];
      }
    }

    for (k = 0; k < 3; k++) {
      for (i4 = 0; i4 < 4; i4++) {
        ptsInCam[k + 3 * i4] = 0.0;
        for (idx = 0; idx < 3; idx++) {
          ptsInCam[k + 3 * i4] += dv3[k + 3 * idx] * b_C_faces[idx + 3 * i4];
        }
      }
    }

    /* find pixels */
    for (jj = 0; jj < 4; jj++) {
      for (k = 0; k < 3; k++) {
        x[k] = 0.0;
        for (i4 = 0; i4 < 3; i4++) {
          x[k] += K[k + 3 * i4] * ptsInCam[i4 + 3 * jj];
        }
      }

      idx = (ii << 2) + jj;
      for (k = 0; k < 2; k++) {
        pts[idx + 24 * k] = rt_roundd_snf(x[k] / x[2]);
      }
    }
  }

  /* write output */
  if (!rtIsNaN(pts[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[0];
  } else {
    ex = pts[idx - 1];
    while (idx + 1 < 25) {
      if (ex > pts[idx]) {
        ex = pts[idx];
      }

      idx++;
    }
  }

  Verts[0] = ex - 1.0;
  if (!rtIsNaN(pts[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[0];
  } else {
    ex = pts[idx - 1];
    while (idx + 1 < 25) {
      if (ex < pts[idx]) {
        ex = pts[idx];
      }

      idx++;
    }
  }

  Verts[1] = ex + 1.0;
  if (!rtIsNaN(pts[24])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k + 23])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[24];
  } else {
    ex = pts[idx + 23];
    while (idx + 1 < 25) {
      if (ex > pts[24 + idx]) {
        ex = pts[24 + idx];
      }

      idx++;
    }
  }

  Verts[2] = ex - 1.0;
  if (!rtIsNaN(pts[24])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k + 23])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[24];
  } else {
    ex = pts[idx + 23];
    while (idx + 1 < 25) {
      if (ex < pts[24 + idx]) {
        ex = pts[24 + idx];
      }

      idx++;
    }
  }

  Verts[3] = ex + 2.0;
}

void locateVerticies(const struct1_T C_faces[6], const double P[3], const double
                     q[4], const double K[9], double Verts[4])
{
  double dv2[9];
  int ii;
  double pts[48];
  int idx;
  int k;
  boolean_T exitg1;
  double ex;
  double b_P[12];
  int i3;
  double b_C_faces[12];
  int jj;
  double ptsInCam[12];
  double x[3];

  /* LOCATEVERTICIES finds the min and max pixels which will be occupied by the */
  /* cube to speed up image generation */
  /*   */
  /*  Inputs */
  /*  C - cube structure */
  /*  P - [3x1] camera position in world frame */
  /*  q = [4x1] quaternion rotating from inertial to camera frame, scalar firsts */
  /*  K = [3x3] camera calibration matrix */
  /*   */
  /*  Outputs */
  /*  Verts = [pixXmin, pixXmax, pixYmin, pixYmax] */
  /* initialize points */
  /* really a cube has 8 unique verticies, but I'm not sure how to identify */
  /* them so we'll go through all of the potentials */
  quat2dcmCoder(q, dv2);
  for (ii = 0; ii < 6; ii++) {
    /* extract points */
    /* translate to camera */
    /* rotate into camera frame */
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
    for (k = 0; k < 3; k++) {
      b_P[k] = P[k];
      b_P[3 + k] = P[k];
      b_P[6 + k] = P[k];
      b_P[9 + k] = P[k];
    }

    for (k = 0; k < 4; k++) {
      for (i3 = 0; i3 < 3; i3++) {
        b_C_faces[i3 + 3 * k] = C_faces[ii].vertex[i3 + 3 * k] - b_P[i3 + 3 * k];
      }
    }

    for (k = 0; k < 3; k++) {
      for (i3 = 0; i3 < 4; i3++) {
        ptsInCam[k + 3 * i3] = 0.0;
        for (idx = 0; idx < 3; idx++) {
          ptsInCam[k + 3 * i3] += dv2[k + 3 * idx] * b_C_faces[idx + 3 * i3];
        }
      }
    }

    /* find pixels */
    for (jj = 0; jj < 4; jj++) {
      for (k = 0; k < 3; k++) {
        x[k] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          x[k] += K[k + 3 * i3] * ptsInCam[i3 + 3 * jj];
        }
      }

      idx = (ii << 2) + jj;
      for (k = 0; k < 2; k++) {
        pts[idx + 24 * k] = rt_roundd_snf(x[k] / x[2]);
      }
    }
  }

  /* write output */
  if (!rtIsNaN(pts[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[0];
  } else {
    ex = pts[idx - 1];
    while (idx + 1 < 25) {
      if (ex > pts[idx]) {
        ex = pts[idx];
      }

      idx++;
    }
  }

  Verts[0] = ex - 1.0;
  if (!rtIsNaN(pts[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[0];
  } else {
    ex = pts[idx - 1];
    while (idx + 1 < 25) {
      if (ex < pts[idx]) {
        ex = pts[idx];
      }

      idx++;
    }
  }

  Verts[1] = ex + 1.0;
  if (!rtIsNaN(pts[24])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k + 23])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[24];
  } else {
    ex = pts[idx + 23];
    while (idx + 1 < 25) {
      if (ex > pts[24 + idx]) {
        ex = pts[24 + idx];
      }

      idx++;
    }
  }

  Verts[2] = ex - 1.0;
  if (!rtIsNaN(pts[24])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 25)) {
      if (!rtIsNaN(pts[k + 23])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = pts[24];
  } else {
    ex = pts[idx + 23];
    while (idx + 1 < 25) {
      if (ex < pts[24 + idx]) {
        ex = pts[24 + idx];
      }

      idx++;
    }
  }

  Verts[3] = ex + 2.0;
}

/* End of code generation (locateVerticies.cpp) */
