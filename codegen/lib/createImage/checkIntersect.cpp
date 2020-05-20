/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * checkIntersect.cpp
 *
 * Code generation for function 'checkIntersect'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "createImage.h"
#include "checkIntersect.h"
#include "norm.h"
#include "strcmp.h"

/* Function Definitions */
void b_checkIntersect(double C_sw, const struct1_T C_faces[6], const double P[3],
                      const double v[3], double *I, double *D)
{
  int ii;
  double ints[6];
  signed char edgeHit[6];
  int i;
  int idx;
  double t;
  int ii_data[6];
  int k;
  boolean_T exitg1;
  double intPt[3];
  double ex;
  int I_data[6];
  int i6;
  boolean_T y;
  boolean_T x_data[6];
  double varargin_1[4];

  /* checkIntersect detirmines if a vector will intersect the cube */
  /*  Inputs */
  /*  C - cube structure */
  /*  P - 3x1 point of origin for the vector */
  /*  v = 3x1 vector direction */
  /*   */
  /*  Outputs */
  /*  I = 0 if no intersect, i if hitting the ith face (1:6), 7 if hitting an */
  /*    edge */
  /*  D = depth to cube, 0 if no intersect */
  /* initialize depth */
  *D = 0.0;

  /* initialize detection logic */
  for (ii = 0; ii < 6; ii++) {
    ints[ii] = 0.0;
    edgeHit[ii] = 0;

    /* extract face information */
    /* our strategy is to solve for when the vector passes through the plane */
    /* of each face */
    if (b_strcmp(C_faces[ii].plane)) {
      t = (C_faces[ii].center[0] - P[0]) / v[0];
    } else if (c_strcmp(C_faces[ii].plane)) {
      t = (C_faces[ii].center[1] - P[1]) / v[1];
    } else {
      /* z plane */
      t = (C_faces[ii].center[2] - P[2]) / v[2];
    }

    /* throw out bad intersects */
    if (rtIsNaN(t) || rtIsInf(t) || (t <= 0.0)) {
    } else {
      /* check to see if the intersect is actually on the face or if is on an */
      /* edge */
      for (i = 0; i < 3; i++) {
        intPt[i] = P[i] + t * v[i];
      }

      if (b_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[1])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[1];
        } else {
          ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[1 + 3 * idx]) {
              ex = C_faces[ii].vertex[1 + 3 * idx];
            }

            idx++;
          }
        }

        if (intPt[1] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[1])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[1];
          } else {
            ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[1 + 3 * idx]) {
                ex = C_faces[ii].vertex[1 + 3 * idx];
              }

              idx++;
            }
          }

          if (intPt[1] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[2])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[2];
            } else {
              ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[2 + 3 * idx]) {
                  ex = C_faces[ii].vertex[2 + 3 * idx];
                }

                idx++;
              }
            }

            if (intPt[2] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[2])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[2];
              } else {
                ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[2 + 3 * idx]) {
                    ex = C_faces[ii].vertex[2 + 3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[2] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else if (c_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[0])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[0];
        } else {
          ex = C_faces[ii].vertex[3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[3 * idx]) {
              ex = C_faces[ii].vertex[3 * idx];
            }

            idx++;
          }
        }

        if (intPt[0] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[0];
          } else {
            ex = C_faces[ii].vertex[3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[3 * idx]) {
                ex = C_faces[ii].vertex[3 * idx];
              }

              idx++;
            }
          }

          if (intPt[0] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[2])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[2];
            } else {
              ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[2 + 3 * idx]) {
                  ex = C_faces[ii].vertex[2 + 3 * idx];
                }

                idx++;
              }
            }

            if (intPt[2] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[2])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[2];
              } else {
                ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[2 + 3 * idx]) {
                    ex = C_faces[ii].vertex[2 + 3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[2] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else if (d_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[1])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[1];
        } else {
          ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[1 + 3 * idx]) {
              ex = C_faces[ii].vertex[1 + 3 * idx];
            }

            idx++;
          }
        }

        if (intPt[1] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[1])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[1];
          } else {
            ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[1 + 3 * idx]) {
                ex = C_faces[ii].vertex[1 + 3 * idx];
              }

              idx++;
            }
          }

          if (intPt[1] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[0])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[0];
            } else {
              ex = C_faces[ii].vertex[3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[3 * idx]) {
                  ex = C_faces[ii].vertex[3 * idx];
                }

                idx++;
              }
            }

            if (intPt[0] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[0])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[0];
              } else {
                ex = C_faces[ii].vertex[3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[3 * idx]) {
                    ex = C_faces[ii].vertex[3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[0] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else {
        ints[ii] = t;
      }
    }
  }

  /* if theres no intersect, return 0 */
  if (norm(ints) == 0.0) {
    *I = 0.0;
  } else {
    ii = 0;
    for (i = 0; i < 6; i++) {
      if (ints[i] > 0.0) {
        ii++;
      }
    }

    idx = 0;
    for (i = 0; i < 6; i++) {
      if (ints[i] > 0.0) {
        ii_data[idx] = i + 1;
        idx++;
      }
    }

    if (ii <= 2) {
      if (ii == 1) {
        *D = ints[ii_data[0] - 1];
      } else if ((ints[ii_data[0] - 1] > ints[ii_data[1] - 1]) || (rtIsNaN
                  (ints[ii_data[0] - 1]) && (!rtIsNaN(ints[ii_data[1] - 1])))) {
        *D = ints[ii_data[1] - 1];
      } else {
        *D = ints[ii_data[0] - 1];
      }
    } else {
      if (!rtIsNaN(ints[ii_data[0] - 1])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= ii)) {
          if (!rtIsNaN(ints[ii_data[k - 1] - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        *D = ints[ii_data[0] - 1];
      } else {
        *D = ints[ii_data[idx - 1] - 1];
        while (idx + 1 <= ii) {
          if (*D > ints[ii_data[idx] - 1]) {
            *D = ints[ii_data[idx] - 1];
          }

          idx++;
        }
      }
    }

    idx = 0;
    ii = 1;
    exitg1 = false;
    while ((!exitg1) && (ii < 7)) {
      if (ints[ii - 1] == *D) {
        idx++;
        ii_data[idx - 1] = ii;
        if (idx >= 6) {
          exitg1 = true;
        } else {
          ii++;
        }
      } else {
        ii++;
      }
    }

    if (1 > idx) {
      i = 0;
    } else {
      i = idx;
    }

    if (0 <= i - 1) {
      memcpy(&I_data[0], &ii_data[0], (unsigned int)(i * (int)sizeof(int)));
    }

    if (1 > idx) {
      i6 = 0;
    } else {
      i6 = idx;
    }

    if (i6 > 1) {
      i = 1;

      /*  disp('Exact Edge Intersect'); */
    }

    /* check for edge hit */
    for (idx = 0; idx < i; idx++) {
      x_data[idx] = (edgeHit[I_data[idx] - 1] == 1);
    }

    y = !(i == 0);
    if (y) {
      k = 1;
      exitg1 = false;
      while ((!exitg1) && (k <= i)) {
        if (!x_data[0]) {
          y = false;
          exitg1 = true;
        } else {
          k = 2;
        }
      }
    }

    if (y) {
      I_data[0] = 7;
    }

    /* assign distance */
    /* make sure I is one dimensional */
    *I = I_data[0];
  }
}

void checkIntersect(double C_sw, const struct1_T C_faces[6], const double P[3],
                    const double v[3], double *I, double *D)
{
  int ii;
  double ints[6];
  signed char edgeHit[6];
  int i;
  int idx;
  double t;
  int ii_data[6];
  int k;
  boolean_T exitg1;
  double intPt[3];
  double ex;
  int I_data[6];
  int i5;
  boolean_T y;
  boolean_T x_data[6];
  double varargin_1[4];

  /* checkIntersect detirmines if a vector will intersect the cube */
  /*  Inputs */
  /*  C - cube structure */
  /*  P - 3x1 point of origin for the vector */
  /*  v = 3x1 vector direction */
  /*   */
  /*  Outputs */
  /*  I = 0 if no intersect, i if hitting the ith face (1:6), 7 if hitting an */
  /*    edge */
  /*  D = depth to cube, 0 if no intersect */
  /* initialize depth */
  *D = 0.0;

  /* initialize detection logic */
  for (ii = 0; ii < 6; ii++) {
    ints[ii] = 0.0;
    edgeHit[ii] = 0;

    /* extract face information */
    /* our strategy is to solve for when the vector passes through the plane */
    /* of each face */
    if (b_strcmp(C_faces[ii].plane)) {
      t = (C_faces[ii].center[0] - P[0]) / v[0];
    } else if (c_strcmp(C_faces[ii].plane)) {
      t = (C_faces[ii].center[1] - P[1]) / v[1];
    } else {
      /* z plane */
      t = (C_faces[ii].center[2] - P[2]) / v[2];
    }

    /* throw out bad intersects */
    if (rtIsNaN(t) || rtIsInf(t) || (t <= 0.0)) {
    } else {
      /* check to see if the intersect is actually on the face or if is on an */
      /* edge */
      for (i = 0; i < 3; i++) {
        intPt[i] = P[i] + t * v[i];
      }

      if (b_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[1])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[1];
        } else {
          ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[1 + 3 * idx]) {
              ex = C_faces[ii].vertex[1 + 3 * idx];
            }

            idx++;
          }
        }

        if (intPt[1] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[1])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[1];
          } else {
            ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[1 + 3 * idx]) {
                ex = C_faces[ii].vertex[1 + 3 * idx];
              }

              idx++;
            }
          }

          if (intPt[1] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[2])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[2];
            } else {
              ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[2 + 3 * idx]) {
                  ex = C_faces[ii].vertex[2 + 3 * idx];
                }

                idx++;
              }
            }

            if (intPt[2] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[2])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[2];
              } else {
                ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[2 + 3 * idx]) {
                    ex = C_faces[ii].vertex[2 + 3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[2] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else if (c_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[0])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[0];
        } else {
          ex = C_faces[ii].vertex[3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[3 * idx]) {
              ex = C_faces[ii].vertex[3 * idx];
            }

            idx++;
          }
        }

        if (intPt[0] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[0];
          } else {
            ex = C_faces[ii].vertex[3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[3 * idx]) {
                ex = C_faces[ii].vertex[3 * idx];
              }

              idx++;
            }
          }

          if (intPt[0] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[2])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[2];
            } else {
              ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[2 + 3 * idx]) {
                  ex = C_faces[ii].vertex[2 + 3 * idx];
                }

                idx++;
              }
            }

            if (intPt[2] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[2])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[2 + 3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[2];
              } else {
                ex = C_faces[ii].vertex[2 + 3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[2 + 3 * idx]) {
                    ex = C_faces[ii].vertex[2 + 3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[2] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[2 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[2] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else if (d_strcmp(C_faces[ii].plane)) {
        if (!rtIsNaN(C_faces[ii].vertex[1])) {
          idx = 1;
        } else {
          idx = 0;
          k = 2;
          exitg1 = false;
          while ((!exitg1) && (k < 5)) {
            if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
              idx = k;
              exitg1 = true;
            } else {
              k++;
            }
          }
        }

        if (idx == 0) {
          ex = C_faces[ii].vertex[1];
        } else {
          ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
          while (idx + 1 < 5) {
            if (ex < C_faces[ii].vertex[1 + 3 * idx]) {
              ex = C_faces[ii].vertex[1 + 3 * idx];
            }

            idx++;
          }
        }

        if (intPt[1] > ex) {
        } else {
          if (!rtIsNaN(C_faces[ii].vertex[1])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k < 5)) {
              if (!rtIsNaN(C_faces[ii].vertex[1 + 3 * (k - 1)])) {
                idx = k;
                exitg1 = true;
              } else {
                k++;
              }
            }
          }

          if (idx == 0) {
            ex = C_faces[ii].vertex[1];
          } else {
            ex = C_faces[ii].vertex[1 + 3 * (idx - 1)];
            while (idx + 1 < 5) {
              if (ex > C_faces[ii].vertex[1 + 3 * idx]) {
                ex = C_faces[ii].vertex[1 + 3 * idx];
              }

              idx++;
            }
          }

          if (intPt[1] < ex) {
          } else {
            if (!rtIsNaN(C_faces[ii].vertex[0])) {
              idx = 1;
            } else {
              idx = 0;
              k = 2;
              exitg1 = false;
              while ((!exitg1) && (k < 5)) {
                if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                  idx = k;
                  exitg1 = true;
                } else {
                  k++;
                }
              }
            }

            if (idx == 0) {
              ex = C_faces[ii].vertex[0];
            } else {
              ex = C_faces[ii].vertex[3 * (idx - 1)];
              while (idx + 1 < 5) {
                if (ex < C_faces[ii].vertex[3 * idx]) {
                  ex = C_faces[ii].vertex[3 * idx];
                }

                idx++;
              }
            }

            if (intPt[0] > ex) {
            } else {
              if (!rtIsNaN(C_faces[ii].vertex[0])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg1 = false;
                while ((!exitg1) && (k < 5)) {
                  if (!rtIsNaN(C_faces[ii].vertex[3 * (k - 1)])) {
                    idx = k;
                    exitg1 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                ex = C_faces[ii].vertex[0];
              } else {
                ex = C_faces[ii].vertex[3 * (idx - 1)];
                while (idx + 1 < 5) {
                  if (ex > C_faces[ii].vertex[3 * idx]) {
                    ex = C_faces[ii].vertex[3 * idx];
                  }

                  idx++;
                }
              }

              if (intPt[0] < ex) {
              } else {
                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[1 + 3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[1] < ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] - C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex < varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] > ex) {
                  edgeHit[ii] = 1;
                }

                for (idx = 0; idx < 4; idx++) {
                  varargin_1[idx] = C_faces[ii].vertex[3 * idx] + C_sw;
                }

                if (!rtIsNaN(varargin_1[0])) {
                  idx = 1;
                } else {
                  idx = 0;
                  k = 2;
                  exitg1 = false;
                  while ((!exitg1) && (k < 5)) {
                    if (!rtIsNaN(varargin_1[k - 1])) {
                      idx = k;
                      exitg1 = true;
                    } else {
                      k++;
                    }
                  }
                }

                if (idx == 0) {
                  ex = varargin_1[0];
                } else {
                  ex = varargin_1[idx - 1];
                  while (idx + 1 < 5) {
                    if (ex > varargin_1[idx]) {
                      ex = varargin_1[idx];
                    }

                    idx++;
                  }
                }

                if (intPt[0] < ex) {
                  edgeHit[ii] = 1;
                }

                ints[ii] = t;
              }
            }
          }
        }
      } else {
        ints[ii] = t;
      }
    }
  }

  /* if theres no intersect, return 0 */
  if (norm(ints) == 0.0) {
    *I = 0.0;
  } else {
    ii = 0;
    for (i = 0; i < 6; i++) {
      if (ints[i] > 0.0) {
        ii++;
      }
    }

    idx = 0;
    for (i = 0; i < 6; i++) {
      if (ints[i] > 0.0) {
        ii_data[idx] = i + 1;
        idx++;
      }
    }

    if (ii <= 2) {
      if (ii == 1) {
        *D = ints[ii_data[0] - 1];
      } else if ((ints[ii_data[0] - 1] > ints[ii_data[1] - 1]) || (rtIsNaN
                  (ints[ii_data[0] - 1]) && (!rtIsNaN(ints[ii_data[1] - 1])))) {
        *D = ints[ii_data[1] - 1];
      } else {
        *D = ints[ii_data[0] - 1];
      }
    } else {
      if (!rtIsNaN(ints[ii_data[0] - 1])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= ii)) {
          if (!rtIsNaN(ints[ii_data[k - 1] - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        *D = ints[ii_data[0] - 1];
      } else {
        *D = ints[ii_data[idx - 1] - 1];
        while (idx + 1 <= ii) {
          if (*D > ints[ii_data[idx] - 1]) {
            *D = ints[ii_data[idx] - 1];
          }

          idx++;
        }
      }
    }

    idx = 0;
    ii = 1;
    exitg1 = false;
    while ((!exitg1) && (ii < 7)) {
      if (ints[ii - 1] == *D) {
        idx++;
        ii_data[idx - 1] = ii;
        if (idx >= 6) {
          exitg1 = true;
        } else {
          ii++;
        }
      } else {
        ii++;
      }
    }

    if (1 > idx) {
      i = 0;
    } else {
      i = idx;
    }

    if (0 <= i - 1) {
      memcpy(&I_data[0], &ii_data[0], (unsigned int)(i * (int)sizeof(int)));
    }

    if (1 > idx) {
      i5 = 0;
    } else {
      i5 = idx;
    }

    if (i5 > 1) {
      i = 1;

      /*  disp('Exact Edge Intersect'); */
    }

    /* check for edge hit */
    for (idx = 0; idx < i; idx++) {
      x_data[idx] = (edgeHit[I_data[idx] - 1] == 1);
    }

    y = !(i == 0);
    if (y) {
      k = 1;
      exitg1 = false;
      while ((!exitg1) && (k <= i)) {
        if (!x_data[0]) {
          y = false;
          exitg1 = true;
        } else {
          k = 2;
        }
      }
    }

    if (y) {
      I_data[0] = 7;
    }

    /* assign distance */
    /* make sure I is one dimensional */
    *I = I_data[0];
  }
}

/* End of code generation (checkIntersect.cpp) */
