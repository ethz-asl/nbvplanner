/*
 * sortrows.cpp
 *
 * Code generation for function 'sortrows'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/sortrows.h"
#include "octomap_to_mesh/OctomapToMesh_emxutil.h"
#include <stdio.h>

/* Function Declarations */
static void apply_row_permutation(emxArray_real_T *y, const emxArray_int32_T
  *idx);
static void eml_sort_idx(const emxArray_real_T *x, const int32_T col[3],
  emxArray_int32_T *idx);
static boolean_T eml_sort_le(const emxArray_real_T *v, const int32_T col[3],
  int32_T irow1, int32_T irow2);

/* Function Definitions */
static void apply_row_permutation(emxArray_real_T *y, const emxArray_int32_T
  *idx)
{
  emxArray_real_T *ycol;
  int32_T m;
  int32_T unnamed_idx_0;
  int32_T i;
  c_emxInit_real_T(&ycol, 1);
  m = y->size[0];
  unnamed_idx_0 = y->size[0];
  i = ycol->size[0];
  ycol->size[0] = unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)ycol, i, (int32_T)sizeof(real_T));
  for (unnamed_idx_0 = 0; unnamed_idx_0 < 3; unnamed_idx_0++) {
    for (i = 0; i + 1 <= m; i++) {
      ycol->data[i] = y->data[(idx->data[i] + y->size[0] * unnamed_idx_0) - 1];
    }

    for (i = 0; i + 1 <= m; i++) {
      y->data[i + y->size[0] * unnamed_idx_0] = ycol->data[i];
    }
  }

  emxFree_real_T(&ycol);
}

static void eml_sort_idx(const emxArray_real_T *x, const int32_T col[3],
  emxArray_int32_T *idx)
{
  int32_T k;
  emxArray_int32_T *idx0;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  k = idx->size[0];
  idx->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)idx, k, (int32_T)sizeof(int32_T));
  for (k = 1; k <= x->size[0]; k++) {
    idx->data[k - 1] = k;
  }

  for (k = 1; k <= x->size[0] - 1; k += 2) {
    if (eml_sort_le(x, col, k, k + 1)) {
    } else {
      idx->data[k - 1] = k + 1;
      idx->data[k] = k;
    }
  }

  emxInit_int32_T(&idx0, 1);
  k = idx0->size[0];
  idx0->size[0] = x->size[0];
  emxEnsureCapacity((emxArray__common *)idx0, k, (int32_T)sizeof(int32_T));
  i = x->size[0];
  for (k = 0; k < i; k++) {
    idx0->data[k] = 1;
  }

  i = 2;
  while (i < x->size[0]) {
    i2 = i << 1;
    j = 1;
    for (pEnd = 1 + i; pEnd < x->size[0] + 1; pEnd = qEnd + i) {
      p = j;
      q = pEnd;
      qEnd = j + i2;
      if (qEnd > x->size[0] + 1) {
        qEnd = x->size[0] + 1;
      }

      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        if (eml_sort_le(x, col, idx->data[p - 1], idx->data[q - 1])) {
          idx0->data[k] = idx->data[p - 1];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              idx0->data[k] = idx->data[q - 1];
              q++;
            }
          }
        } else {
          idx0->data[k] = idx->data[q - 1];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              idx0->data[k] = idx->data[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (k = 0; k + 1 <= kEnd; k++) {
        idx->data[(j + k) - 1] = idx0->data[k];
      }

      j = qEnd;
    }

    i = i2;
  }

  emxFree_int32_T(&idx0);
}

static boolean_T eml_sort_le(const emxArray_real_T *v, const int32_T col[3],
  int32_T irow1, int32_T irow2)
{
  boolean_T p;
  int32_T k;
  boolean_T exitg1;
  int32_T coloffset;
  boolean_T b0;
  p = TRUE;
  k = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (k < 3)) {
    coloffset = (col[k] - 1) * v->size[0] - 1;
    if ((v->data[coloffset + irow1] == v->data[coloffset + irow2]) || (rtIsNaN
         (v->data[coloffset + irow1]) && rtIsNaN(v->data[coloffset + irow2]))) {
      b0 = TRUE;
    } else {
      b0 = FALSE;
    }

    if (!b0) {
      if ((v->data[coloffset + irow1] <= v->data[coloffset + irow2]) || rtIsNaN
          (v->data[coloffset + irow2])) {
        p = TRUE;
      } else {
        p = FALSE;
      }

      exitg1 = TRUE;
    } else {
      k++;
    }
  }

  return p;
}

void sortrows(emxArray_real_T *y, emxArray_real_T *ndx)
{
  int32_T m;
  int32_T k;
  int32_T col[3];
  emxArray_int32_T *idx;
  m = y->size[0];
  k = ndx->size[0];
  ndx->size[0] = y->size[0];
  emxEnsureCapacity((emxArray__common *)ndx, k, (int32_T)sizeof(real_T));
  for (k = 0; k < 3; k++) {
    col[k] = k + 1;
  }

  emxInit_int32_T(&idx, 1);
  eml_sort_idx(y, col, idx);
  apply_row_permutation(y, idx);
  for (k = 0; k + 1 <= m; k++) {
    ndx->data[k] = idx->data[k];
  }

  emxFree_int32_T(&idx);
}

/* End of code generation (sortrows.cpp) */
