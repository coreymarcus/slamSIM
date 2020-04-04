/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_createImage_api.c
 *
 * Code generation for function '_coder_createImage_api'
 *
 */

/* Include files */
#include "tmwtypes.h"
#include "_coder_createImage_api.h"
#include "_coder_createImage_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "createImage",                       /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static char_T ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, cell_0 *y);
static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[12]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y);
static boolean_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static real_T (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *C, const
  char_T *identifier, cell_0 *y);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);
static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3]);
static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[921600];
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[6]);
static real_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2];
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y);
static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static char_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12]);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y);
static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char_T *identifier))[3];
static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4];
static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *V, const
  char_T *identifier))[921600];
static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[921600];
static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *sz,
  const char_T *identifier))[2];
static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2];
static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *K, const
  char_T *identifier))[9];
static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);
static real_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3]);

/* Function Definitions */
static char_T ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  char_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "char",
    false, 0U, (int32_T *)&dims);
  emlrtImportCharR2015b(sp, src, &ret);
  emlrtDestroyArray(&src);
  return ret;
}

static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, cell_0 *y)
{
  emlrtMsgIdentifier thisId;
  int32_T iv0[1];
  boolean_T bv0[1];
  thisId.fParent = parentId;
  thisId.bParentIsCell = true;
  iv0[0] = 11;
  bv0[0] = false;
  emlrtCheckCell(sp, parentId, u, 1U, iv0, bv0);
  thisId.fIdentifier = "1";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 0)), &thisId,
                     &y->f1);
  thisId.fIdentifier = "2";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 1)), &thisId,
                     &y->f2);
  thisId.fIdentifier = "3";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 2)), &thisId,
                     &y->f3);
  thisId.fIdentifier = "4";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 3)), &thisId,
                     &y->f4);
  thisId.fIdentifier = "5";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 4)), &thisId,
                     &y->f5);
  thisId.fIdentifier = "6";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 5)), &thisId,
                     &y->f6);
  thisId.fIdentifier = "7";
  k_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 6)), &thisId,
                     &y->f7);
  thisId.fIdentifier = "8";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 7)), &thisId,
                     &y->f8);
  thisId.fIdentifier = "9";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 8)), &thisId,
                     &y->f9);
  thisId.fIdentifier = "10";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 9)), &thisId,
                     &y->f10);
  thisId.fIdentifier = "11";
  c_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, 10)), &thisId,
                     &y->f11);
  emlrtDestroyArray(&u);
}

static void bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[12])
{
  static const int32_T dims[2] = { 3, 4 };

  int32_T i3;
  int32_T i4;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i3 = 0; i3 < 4; i3++) {
    for (i4 = 0; i4 < 3; i4++) {
      ret[i4 + 3 * i3] = (*(real_T (*)[12])emlrtMxGetData(src))[i4 + 3 * i3];
    }
  }

  emlrtDestroyArray(&src);
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[5] = { "P", "s", "sw", "ec", "faces" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 5, fieldNames, 0U, (int32_T *)&dims);
  thisId.fIdentifier = "P";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "P")),
                     &thisId, y->P);
  thisId.fIdentifier = "s";
  y->s = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "s")),
    &thisId);
  thisId.fIdentifier = "sw";
  y->sw = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2,
    "sw")), &thisId);
  thisId.fIdentifier = "ec";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "ec")),
                     &thisId, y->ec);
  thisId.fIdentifier = "faces";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 4, "faces")),
                     &thisId, y->faces);
  emlrtDestroyArray(&u);
}

static boolean_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "logical",
    false, 0U, (int32_T *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  w_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  real_T (*ret)[3];
  static const int32_T dims[1] = { 3 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  real_T (*ret)[4];
  static const int32_T dims[1] = { 4 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  ret = (real_T (*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *C, const
  char_T *identifier, cell_0 *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(C), &thisId, y);
  emlrtDestroyArray(&C);
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv2[3] = { 0, 0, 0 };

  y = NULL;
  m0 = emlrtCreateNumericArray(3, iv2, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u->data[0]);
  emlrtSetDimensions((mxArray *)m0, u->size, 3);
  emlrtAssign(&y, m0);
  return y;
}

static void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      emlrtFreeMex2018a(sp, (void *)(*pEmxArray)->data);
    }

    emlrtFreeMex2018a(sp, (void *)(*pEmxArray)->size);
    emlrtFreeMex2018a(sp, (void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

static void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, boolean_T doPush)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)emlrtMallocMex(sizeof(emxArray_real_T));
  if (doPush) {
    emlrtPushHeapReferenceStackR2018a(sp, (void *)pEmxArray, (void (*)(const
      void *, void *))emxFree_real_T);
  }

  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)emlrtMallocMex(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[3])
{
  y_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[921600]
{
  real_T (*ret)[921600];
  static const int32_T dims[3] = { 480, 640, 3 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 3U, *(int32_T (*)[3])&dims[0]);
  ret = (real_T (*)[921600])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T y[6])
{
  emlrtMsgIdentifier thisId;
  int32_T iv1[1];
  boolean_T bv1[1];
  int32_T i0;
  char_T str[11];
  thisId.fParent = parentId;
  thisId.bParentIsCell = true;
  iv1[0] = 6;
  bv1[0] = false;
  emlrtCheckCell(sp, parentId, u, 1U, iv1, bv1);
  for (i0 = 0; i0 < 6; i0++) {
    sprintf(&str[0], "%d", i0 + 1);
    thisId.fIdentifier = &str[0];
    h_emlrt_marshallIn(sp, emlrtAlias(emlrtGetCell(sp, parentId, u, i0)),
                       &thisId, &y[i0]);
  }

  emlrtDestroyArray(&u);
}

static real_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2]
{
  real_T (*ret)[2];
  static const int32_T dims[2] = { 1, 2 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  ret = (real_T (*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[4] = { "color", "plane", "center", "vertex" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 4, fieldNames, 0U, (int32_T *)&dims);
  thisId.fIdentifier = "color";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "color")),
                     &thisId, y->color);
  thisId.fIdentifier = "plane";
  y->plane = i_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1,
    "plane")), &thisId);
  thisId.fIdentifier = "center";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2, "center")),
                     &thisId, y->center);
  thisId.fIdentifier = "vertex";
  j_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "vertex")),
                     &thisId, y->vertex);
  emlrtDestroyArray(&u);
}

static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  real_T (*ret)[9];
  static const int32_T dims[2] = { 3, 3 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  ret = (real_T (*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static char_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  char_T y;
  y = ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[12])
{
  bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  static const char * fieldNames[6] = { "P", "s", "sw", "ec", "isAllEncomp",
    "faces" };

  static const int32_T dims = 0;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b(sp, parentId, u, 6, fieldNames, 0U, (int32_T *)&dims);
  thisId.fIdentifier = "P";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 0, "P")),
                     &thisId, y->P);
  thisId.fIdentifier = "s";
  d_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 1, "s")),
                     &thisId, y->s);
  thisId.fIdentifier = "sw";
  y->sw = e_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 2,
    "sw")), &thisId);
  thisId.fIdentifier = "ec";
  f_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 3, "ec")),
                     &thisId, y->ec);
  thisId.fIdentifier = "isAllEncomp";
  y->isAllEncomp = l_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u,
    0, 4, "isAllEncomp")), &thisId);
  thisId.fIdentifier = "faces";
  g_emlrt_marshallIn(sp, emlrtAlias(emlrtGetFieldR2017b(sp, u, 0, 5, "faces")),
                     &thisId, y->faces);
  emlrtDestroyArray(&u);
}

static boolean_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char_T *identifier))[3]
{
  real_T (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = n_emlrt_marshallIn(sp, emlrtAlias(P), &thisId);
  emlrtDestroyArray(&P);
  return y;
}
  static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  real_T (*y)[3];
  y = db_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *q, const
  char_T *identifier))[4]
{
  real_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = p_emlrt_marshallIn(sp, emlrtAlias(q), &thisId);
  emlrtDestroyArray(&q);
  return y;
}
  static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  real_T (*y)[4];
  y = eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *V, const
  char_T *identifier))[921600]
{
  real_T (*y)[921600];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = r_emlrt_marshallIn(sp, emlrtAlias(V), &thisId);
  emlrtDestroyArray(&V);
  return y;
}
  static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[921600]
{
  real_T (*y)[921600];
  y = fb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *sz,
  const char_T *identifier))[2]
{
  real_T (*y)[2];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = t_emlrt_marshallIn(sp, emlrtAlias(sz), &thisId);
  emlrtDestroyArray(&sz);
  return y;
}
  static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[2]
{
  real_T (*y)[2];
  y = gb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *K, const
  char_T *identifier))[9]
{
  real_T (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = v_emlrt_marshallIn(sp, emlrtAlias(K), &thisId);
  emlrtDestroyArray(&K);
  return y;
}
  static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  real_T (*y)[9];
  y = hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  int32_T i1;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i1 = 0; i1 < 3; i1++) {
    ret[i1] = (*(real_T (*)[3])emlrtMxGetData(src))[i1];
  }

  emlrtDestroyArray(&src);
}

static real_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 0U, (int32_T *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[2] = { 1, 3 };

  int32_T i2;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i2 = 0; i2 < 3; i2++) {
    ret[i2] = (*(real_T (*)[3])emlrtMxGetData(src))[i2];
  }

  emlrtDestroyArray(&src);
}

void createImage_api(const mxArray * const prhs[6], int32_T nlhs, const mxArray *
                     plhs[1])
{
  emxArray_real_T *I;
  cell_0 C;
  real_T (*P)[3];
  real_T (*q)[4];
  real_T (*V)[921600];
  real_T (*sz)[2];
  real_T (*K)[9];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  (void)nlhs;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &I, 3, true);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "C", &C);
  P = m_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "P");
  q = o_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "q");
  V = q_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "V");
  sz = s_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "sz");
  K = u_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "K");

  /* Invoke the target function */
  createImage(&C, *P, *q, *V, *sz, *K, I);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(I);
  I->canFreeData = false;
  emxFree_real_T(&st, &I);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

void createImage_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  createImage_xil_terminate();
}

void createImage_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void createImage_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (_coder_createImage_api.c) */
