/*
 * ind2sub.cpp
 *
 * Code generation for function 'ind2sub'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/ind2sub.h"
#include "octomap_to_mesh/OctomapToMesh_emxutil.h"
#include "octomap_to_mesh/OctomapToMesh_rtwutil.h"
#include <stdio.h>

/* Function Definitions */
void b_ind2sub(const real_T siz[3], real_T ndx, real_T *varargout_1, real_T
               *varargout_2, real_T *varargout_3)
{
  int32_T cpsiz[2];
  int32_T vk;
  int32_T v1;
  for (vk = 0; vk < 2; vk++) {
    cpsiz[vk] = (int32_T)siz[vk];
  }

  cpsiz[1] *= cpsiz[0];
  vk = div_s32((int32_T)ndx - 1, cpsiz[1]);
  *varargout_3 = vk + 1;
  v1 = (int32_T)ndx - vk * cpsiz[1];
  vk = div_s32(v1 - 1, cpsiz[0]);
  *varargout_2 = vk + 1;
  v1 -= vk * cpsiz[0];
  *varargout_1 = v1;
}

void ind2sub(const real_T siz[3], const emxArray_real_T *ndx, emxArray_real_T
             *varargout_1, emxArray_real_T *varargout_2, emxArray_real_T
             *varargout_3)
{
  int32_T cpsiz[2];
  int32_T i5;
  emxArray_int32_T *v1;
  int32_T loop_ub;
  emxArray_int32_T *vk;
  for (i5 = 0; i5 < 2; i5++) {
    cpsiz[i5] = (int32_T)siz[i5];
  }

  emxInit_int32_T(&v1, 1);
  cpsiz[1] *= cpsiz[0];
  i5 = v1->size[0];
  v1->size[0] = ndx->size[0];
  emxEnsureCapacity((emxArray__common *)v1, i5, (int32_T)sizeof(int32_T));
  loop_ub = ndx->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    v1->data[i5] = (int32_T)ndx->data[i5] - 1;
  }

  emxInit_int32_T(&vk, 1);
  i5 = vk->size[0];
  vk->size[0] = v1->size[0];
  emxEnsureCapacity((emxArray__common *)vk, i5, (int32_T)sizeof(int32_T));
  loop_ub = v1->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    vk->data[i5] = div_s32(v1->data[i5], cpsiz[1]);
  }

  i5 = varargout_3->size[0];
  varargout_3->size[0] = vk->size[0];
  emxEnsureCapacity((emxArray__common *)varargout_3, i5, (int32_T)sizeof(real_T));
  loop_ub = vk->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    varargout_3->data[i5] = vk->data[i5] + 1;
  }

  i5 = v1->size[0];
  emxEnsureCapacity((emxArray__common *)v1, i5, (int32_T)sizeof(int32_T));
  loop_ub = v1->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    v1->data[i5] -= vk->data[i5] * cpsiz[1];
  }

  i5 = vk->size[0];
  vk->size[0] = v1->size[0];
  emxEnsureCapacity((emxArray__common *)vk, i5, (int32_T)sizeof(int32_T));
  loop_ub = v1->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    vk->data[i5] = div_s32(v1->data[i5], cpsiz[0]);
  }

  i5 = varargout_2->size[0];
  varargout_2->size[0] = vk->size[0];
  emxEnsureCapacity((emxArray__common *)varargout_2, i5, (int32_T)sizeof(real_T));
  loop_ub = vk->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    varargout_2->data[i5] = vk->data[i5] + 1;
  }

  i5 = v1->size[0];
  emxEnsureCapacity((emxArray__common *)v1, i5, (int32_T)sizeof(int32_T));
  loop_ub = v1->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    v1->data[i5] -= vk->data[i5] * cpsiz[0];
  }

  emxFree_int32_T(&vk);
  i5 = varargout_1->size[0];
  varargout_1->size[0] = v1->size[0];
  emxEnsureCapacity((emxArray__common *)varargout_1, i5, (int32_T)sizeof(real_T));
  loop_ub = v1->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    varargout_1->data[i5] = v1->data[i5] + 1;
  }

  emxFree_int32_T(&v1);
}

/* End of code generation (ind2sub.cpp) */
