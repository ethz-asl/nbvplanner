/*
 * cat.cpp
 *
 * Code generation for function 'cat'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/cat.h"
#include "octomap_to_mesh/OctomapToMesh_emxutil.h"
#include <stdio.h>

/* Function Declarations */
static void b_catsize(const int32_T varargin_1_size[3], const emxArray_real_T
                      *varargin_2, real_T sz[3]);
static void c_catsize(const int32_T varargin_1_size[3], const emxArray_boolean_T
                      *varargin_2, real_T sz[3]);
static void catsize(const int32_T varargin_1_size[3], const emxArray_boolean_T
                    *varargin_2, real_T sz[3]);
static void d_catsize(const int32_T varargin_1_size[3], const emxArray_real_T
                      *varargin_2, real_T sz[3]);
static void e_catsize(const int32_T varargin_1_size[2], const emxArray_boolean_T
                      *varargin_2, real_T sz[3]);
static void f_catsize(const int32_T varargin_1_size[2], const emxArray_real_T
                      *varargin_2, real_T sz[3]);

/* Function Definitions */
static void b_catsize(const int32_T varargin_1_size[3], const emxArray_real_T
                      *varargin_2, real_T sz[3])
{
  int32_T i7;
  for (i7 = 0; i7 < 3; i7++) {
    sz[i7] = (int8_T)varargin_1_size[i7];
  }

  sz[0] += (real_T)varargin_2->size[0];
  sz[0]++;
}

static void c_catsize(const int32_T varargin_1_size[3], const emxArray_boolean_T
                      *varargin_2, real_T sz[3])
{
  int32_T i8;
  for (i8 = 0; i8 < 3; i8++) {
    sz[i8] = (int8_T)varargin_1_size[i8];
  }

  sz[1] += (real_T)varargin_2->size[1];
  sz[1]++;
}

static void catsize(const int32_T varargin_1_size[3], const emxArray_boolean_T
                    *varargin_2, real_T sz[3])
{
  int32_T i6;
  for (i6 = 0; i6 < 3; i6++) {
    sz[i6] = (int8_T)varargin_1_size[i6];
  }

  sz[0] += (real_T)varargin_2->size[0];
  sz[0]++;
}

static void d_catsize(const int32_T varargin_1_size[3], const emxArray_real_T
                      *varargin_2, real_T sz[3])
{
  int32_T i9;
  for (i9 = 0; i9 < 3; i9++) {
    sz[i9] = (int8_T)varargin_1_size[i9];
  }

  sz[1] += (real_T)varargin_2->size[1];
  sz[1]++;
}

static void e_catsize(const int32_T varargin_1_size[2], const emxArray_boolean_T
                      *varargin_2, real_T sz[3])
{
  int32_T i11;
  for (i11 = 0; i11 < 3; i11++) {
    sz[i11] = 1.0;
  }

  for (i11 = 0; i11 < 2; i11++) {
    sz[i11] = (int8_T)varargin_1_size[i11];
  }

  sz[2] = 1.0 + (real_T)varargin_2->size[2];
  sz[2]++;
}

static void f_catsize(const int32_T varargin_1_size[2], const emxArray_real_T
                      *varargin_2, real_T sz[3])
{
  int32_T i13;
  for (i13 = 0; i13 < 3; i13++) {
    sz[i13] = 1.0;
  }

  for (i13 = 0; i13 < 2; i13++) {
    sz[i13] = (int8_T)varargin_1_size[i13];
  }

  sz[2] = 1.0 + (real_T)varargin_2->size[2];
  sz[2]++;
}

void b_cat(const int32_T varargin_1_size[3], const emxArray_real_T *varargin_2,
           emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T iy;
  int32_T npages;
  int32_T b_ysize[3];
  int32_T i;
  int32_T l;
  b_catsize(varargin_1_size, varargin_2, ysize);
  iy = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, iy, (int32_T)sizeof(real_T));
  npages = 1;
  for (iy = 0; iy < 2; iy++) {
    b_ysize[0] = (int32_T)ysize[0];
    b_ysize[1] = (int32_T)ysize[1];
    b_ysize[2] = (int32_T)ysize[2];
    npages *= b_ysize[iy + 1];
  }

  iy = 1;
  for (i = 0; i < npages; i++) {
    y->data[iy - 1] = 0.0;
    for (l = 1; l <= varargin_2->size[0]; l++) {
      y->data[iy] = 0.0;
      iy++;
    }

    y->data[iy] = 0.0;
    iy += 2;
  }
}

void c_cat(const int32_T varargin_1_size[3], const emxArray_boolean_T
           *varargin_2, emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T iy;
  int8_T unnamed_idx_0;
  int32_T i;
  int32_T istart;
  int32_T j;
  int32_T ix;
  int32_T l;
  c_catsize(varargin_1_size, varargin_2, ysize);
  iy = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, iy, (int32_T)sizeof(real_T));
  unnamed_idx_0 = (int8_T)(int32_T)ysize[0];
  ysize[2] = (int32_T)ysize[2];
  iy = -1;
  for (i = 0; i < (int32_T)ysize[2]; i++) {
    istart = iy;
    for (j = 0; j + 1 <= unnamed_idx_0; j++) {
      istart++;
      y->data[istart] = 0.0;
      iy = istart + unnamed_idx_0;
      ix = j + i * (unnamed_idx_0 * varargin_2->size[1]);
      for (l = 1; l <= varargin_2->size[1]; l++) {
        y->data[iy] = varargin_2->data[ix];
        ix += unnamed_idx_0;
        iy += unnamed_idx_0;
      }

      y->data[iy] = 0.0;
      iy += unnamed_idx_0;
    }

    iy -= unnamed_idx_0;
  }
}

void cat(const int32_T varargin_1_size[3], const emxArray_boolean_T *varargin_2,
         emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T iy;
  int32_T npages;
  int32_T b_ysize[3];
  int32_T i;
  int32_T ix;
  int32_T l;
  catsize(varargin_1_size, varargin_2, ysize);
  iy = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, iy, (int32_T)sizeof(real_T));
  npages = 1;
  for (iy = 0; iy < 2; iy++) {
    b_ysize[0] = (int32_T)ysize[0];
    b_ysize[1] = (int32_T)ysize[1];
    b_ysize[2] = (int32_T)ysize[2];
    npages *= b_ysize[iy + 1];
  }

  iy = 1;
  for (i = 0; i < npages; i++) {
    y->data[iy - 1] = 0.0;
    ix = i * varargin_2->size[0];
    for (l = 1; l <= varargin_2->size[0]; l++) {
      y->data[iy] = varargin_2->data[ix];
      ix++;
      iy++;
    }

    y->data[iy] = 0.0;
    iy += 2;
  }
}

void d_cat(const int32_T varargin_1_size[3], const emxArray_real_T *varargin_2,
           emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T iy;
  uint8_T unnamed_idx_0;
  int32_T i;
  int32_T istart;
  int32_T j;
  int32_T ix;
  int32_T l;
  d_catsize(varargin_1_size, varargin_2, ysize);
  iy = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, iy, (int32_T)sizeof(real_T));
  unnamed_idx_0 = (uint8_T)(int32_T)ysize[0];
  ysize[2] = (int32_T)ysize[2];
  iy = -1;
  for (i = 0; i < (int32_T)ysize[2]; i++) {
    istart = iy;
    for (j = 0; j + 1 <= unnamed_idx_0; j++) {
      istart++;
      y->data[istart] = 0.0;
      iy = istart + unnamed_idx_0;
      ix = j + i * (unnamed_idx_0 * varargin_2->size[1]);
      for (l = 1; l <= varargin_2->size[1]; l++) {
        y->data[iy] = varargin_2->data[ix];
        ix += unnamed_idx_0;
        iy += unnamed_idx_0;
      }

      y->data[iy] = 0.0;
      iy += unnamed_idx_0;
    }

    iy -= unnamed_idx_0;
  }
}

void e_cat(const int32_T varargin_1_size[2], const emxArray_boolean_T
           *varargin_2, const int32_T varargin_3_size[2], emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T i10;
  int32_T iy;
  int32_T j;
  e_catsize(varargin_1_size, varargin_2, ysize);
  i10 = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, i10, (int32_T)sizeof(real_T));
  iy = -1;
  i10 = varargin_1_size[0] * varargin_1_size[1];
  for (j = 1; j <= i10; j++) {
    iy++;
    y->data[iy] = 0.0;
  }

  i10 = varargin_2->size[0] * varargin_2->size[1] * varargin_2->size[2];
  for (j = 1; j <= i10; j++) {
    iy++;
    y->data[iy] = varargin_2->data[j - 1];
  }

  i10 = varargin_3_size[0] * varargin_3_size[1];
  for (j = 1; j <= i10; j++) {
    iy++;
    y->data[iy] = 0.0;
  }
}

void f_cat(const int32_T varargin_1_size[2], const emxArray_real_T *varargin_2,
           const int32_T varargin_3_size[2], emxArray_real_T *y)
{
  real_T ysize[3];
  int32_T i12;
  int32_T iy;
  int32_T j;
  f_catsize(varargin_1_size, varargin_2, ysize);
  i12 = y->size[0] * y->size[1] * y->size[2];
  y->size[0] = (int32_T)ysize[0];
  y->size[1] = (int32_T)ysize[1];
  y->size[2] = (int32_T)ysize[2];
  emxEnsureCapacity((emxArray__common *)y, i12, (int32_T)sizeof(real_T));
  iy = -1;
  i12 = varargin_1_size[0] * varargin_1_size[1];
  for (j = 1; j <= i12; j++) {
    iy++;
    y->data[iy] = 0.0;
  }

  i12 = varargin_2->size[0] * varargin_2->size[1] * varargin_2->size[2];
  for (j = 1; j <= i12; j++) {
    iy++;
    y->data[iy] = varargin_2->data[j - 1];
  }

  i12 = varargin_3_size[0] * varargin_3_size[1];
  for (j = 1; j <= i12; j++) {
    iy++;
    y->data[iy] = 0.0;
  }
}

/* End of code generation (cat.cpp) */
