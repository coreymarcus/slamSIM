/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * quat2dcmCoder.cpp
 *
 * Code generation for function 'quat2dcmCoder'
 *
 */

/* Include files */
#include <math.h>
#include "rt_nonfinite.h"
#include "createImage.h"
#include "quat2dcmCoder.h"

/* Function Definitions */
void quat2dcmCoder(const double q[4], double dcm[9])
{
  int k;
  double qm;
  double z1[4];
  double qin[4];

  /*   QUAT2DCM Convert quaternion to direction cosine matrix. */
  /*    N = QUAT2DCM( Q ) calculates the direction cosine matrix, N, for a */
  /*    given quaternion, Q.  Input Q is an M-by-4 matrix containing M */
  /*    quaternions.  N returns a 3-by-3-by-M matrix of direction cosine  */
  /*    matrices.  The direction cosine matrix performs the coordinate */
  /*    transformation of a vector in inertial axes to a vector in body axes. */
  /*    Each element of Q must be a real number.  Additionally, Q has its */
  /*    scalar number as the first column.  */
  /*  */
  /*    Examples: */
  /*  */
  /*    Determine the direction cosine matrix from q = [1 0 1 0]: */
  /*       dcm = quat2dcm([1 0 1 0]) */
  /*  */
  /*    Determine the direction cosine matrices from multiple quaternions: */
  /*       q = [1 0 1 0; 1 0.5 0.3 0.1]; */
  /*       dcm = quat2dcm(q) */
  /*  */
  /*    See also ANGLE2DCM, DCM2ANGLE, DCM2QUAT, ANGLE2QUAT, QUAT2ANGLE, QUATROTATE.  */
  /*    Copyright 2000-2007 The MathWorks, Inc. */
  for (k = 0; k < 4; k++) {
    qin[k] = q[k];
    z1[k] = q[k] * q[k];
  }

  qm = z1[0];
  for (k = 0; k < 3; k++) {
    qm += z1[k + 1];
  }

  qm = sqrt(qm);
  for (k = 0; k < 4; k++) {
    qin[k] /= qm;
  }

  dcm[0] = ((qin[0] * qin[0] + qin[1] * qin[1]) - qin[2] * qin[2]) - qin[3] *
    qin[3];
  dcm[3] = 2.0 * (qin[1] * qin[2] + qin[0] * qin[3]);
  dcm[6] = 2.0 * (qin[1] * qin[3] - qin[0] * qin[2]);
  dcm[1] = 2.0 * (qin[1] * qin[2] - qin[0] * qin[3]);
  dcm[4] = ((qin[0] * qin[0] - qin[1] * qin[1]) + qin[2] * qin[2]) - qin[3] *
    qin[3];
  dcm[7] = 2.0 * (qin[2] * qin[3] + qin[0] * qin[1]);
  dcm[2] = 2.0 * (qin[1] * qin[3] + qin[0] * qin[2]);
  dcm[5] = 2.0 * (qin[2] * qin[3] - qin[0] * qin[1]);
  dcm[8] = ((qin[0] * qin[0] - qin[1] * qin[1]) - qin[2] * qin[2]) + qin[3] *
    qin[3];
}

/* End of code generation (quat2dcmCoder.cpp) */
