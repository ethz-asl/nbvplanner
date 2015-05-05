/*
 * retrieve_meshformat.cpp
 *
 * Code generation for function 'retrieve_meshformat'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/retrieve_meshformat.h"
#include "octomap_to_mesh/OctomapToMesh_emxutil.h"
#include "octomap_to_mesh/unique.h"
#include "octomap_to_mesh/sortrows.h"
#include <stdio.h>

/* Function Declarations */
static int32_T compute_nones(const emxArray_boolean_T *x, int32_T n);
static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y);

/* Function Definitions */
static int32_T compute_nones(const emxArray_boolean_T *x, int32_T n)
{
  int32_T k;
  int32_T i;
  k = 0;
  for (i = 1; i <= n; i++) {
    if (x->data[i - 1]) {
      k++;
    }
  }

  return k;
}

static void eml_li_find(const emxArray_boolean_T *x, emxArray_int32_T *y)
{
  int32_T k;
  int32_T i;
  k = compute_nones(x, x->size[0]);
  i = y->size[0];
  y->size[0] = k;
  emxEnsureCapacity((emxArray__common *)y, i, (int32_T)sizeof(int32_T));
  k = 0;
  for (i = 1; i <= x->size[0]; i++) {
    if (x->data[i - 1]) {
      y->data[k] = i;
      k++;
    }
  }
}

void retrieve_meshformat(const emxArray_real_T *meshXYZ, emxArray_real_T *faces,
  emxArray_real_T *vertices)
{
  int32_T loop_ub;
  int32_T nb;
  int32_T k;
  int32_T i14;
  int32_T k0;
  emxArray_real_T *a;
  emxArray_real_T *b_vertices;
  int32_T j;
  emxArray_real_T *c_vertices;
  emxArray_boolean_T *x;
  emxArray_real_T *idx;
  emxArray_int32_T *ii;
  emxArray_boolean_T *d_vertices;
  emxArray_boolean_T *e_vertices;
  emxArray_int32_T *b_ii;
  emxArray_real_T *b_idx;
  emxArray_real_T *c_idx;
  real_T b_meshXYZ;
  boolean_T exitg1;
  boolean_T guard1 = FALSE;

  /*  */
  /*    Syntax: */
  /*    [faces,vertices] = retrieve_meshformat(meshXYZ) */
  /*  */
  /*    Inputs: */
  /*    meshXYZ     : mesh xyz struct */
  /*     */
  /*    Outputs: */
  /*    faces       : mesh faces */
  /*    vertices    : mesh vertices */
  /*  */
  /*    Authors: Kostas Alexis (konstantinos.alexis@mavt.ethz.ch) */
  /*  */
  loop_ub = meshXYZ->size[0];
  nb = meshXYZ->size[0];
  k = meshXYZ->size[0];
  i14 = vertices->size[0] * vertices->size[1];
  vertices->size[0] = (loop_ub + nb) + k;
  vertices->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)vertices, i14, (int32_T)sizeof(real_T));
  for (i14 = 0; i14 < 3; i14++) {
    for (k0 = 0; k0 < loop_ub; k0++) {
      vertices->data[k0 + vertices->size[0] * i14] = meshXYZ->data[k0 +
        meshXYZ->size[0] * i14];
    }
  }

  for (i14 = 0; i14 < 3; i14++) {
    for (k0 = 0; k0 < nb; k0++) {
      vertices->data[(k0 + loop_ub) + vertices->size[0] * i14] = meshXYZ->data
        [(k0 + meshXYZ->size[0] * i14) + meshXYZ->size[0] * meshXYZ->size[1]];
    }
  }

  for (i14 = 0; i14 < 3; i14++) {
    for (k0 = 0; k0 < k; k0++) {
      vertices->data[((k0 + loop_ub) + nb) + vertices->size[0] * i14] =
        meshXYZ->data[(k0 + meshXYZ->size[0] * i14) + (meshXYZ->size[0] *
        meshXYZ->size[1] << 1)];
    }
  }

  b_emxInit_real_T(&a, 2);
  i14 = a->size[0] * a->size[1];
  a->size[0] = vertices->size[0];
  a->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)a, i14, (int32_T)sizeof(real_T));
  loop_ub = vertices->size[0] * vertices->size[1];
  for (i14 = 0; i14 < loop_ub; i14++) {
    a->data[i14] = vertices->data[i14];
  }

  if (vertices->size[0] == 0) {
  } else {
    c_emxInit_real_T(&b_vertices, 1);
    sortrows(vertices, b_vertices);
    nb = 0;
    k = 1;
    emxFree_real_T(&b_vertices);
    while (k <= a->size[0]) {
      k0 = k;
      do {
        k++;
      } while (!((k > a->size[0]) || rows_differ(vertices, k0, k)));

      nb++;
      for (j = 0; j < 3; j++) {
        vertices->data[(nb + vertices->size[0] * j) - 1] = vertices->data[(k0 +
          vertices->size[0] * j) - 1];
      }
    }

    if (1 > nb) {
      loop_ub = 0;
    } else {
      loop_ub = nb;
    }

    b_emxInit_real_T(&c_vertices, 2);
    i14 = c_vertices->size[0] * c_vertices->size[1];
    c_vertices->size[0] = loop_ub;
    c_vertices->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)c_vertices, i14, (int32_T)sizeof
                      (real_T));
    for (i14 = 0; i14 < 3; i14++) {
      for (k0 = 0; k0 < loop_ub; k0++) {
        c_vertices->data[k0 + c_vertices->size[0] * i14] = vertices->data[k0 +
          vertices->size[0] * i14];
      }
    }

    i14 = vertices->size[0] * vertices->size[1];
    vertices->size[0] = c_vertices->size[0];
    vertices->size[1] = 3;
    emxEnsureCapacity((emxArray__common *)vertices, i14, (int32_T)sizeof(real_T));
    for (i14 = 0; i14 < 3; i14++) {
      loop_ub = c_vertices->size[0];
      for (k0 = 0; k0 < loop_ub; k0++) {
        vertices->data[k0 + vertices->size[0] * i14] = c_vertices->data[k0 +
          c_vertices->size[0] * i14];
      }
    }

    emxFree_real_T(&c_vertices);
  }

  emxFree_real_T(&a);
  nb = meshXYZ->size[0];
  i14 = faces->size[0] * faces->size[1];
  faces->size[0] = nb;
  faces->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)faces, i14, (int32_T)sizeof(real_T));
  loop_ub = meshXYZ->size[0] * 3;
  for (i14 = 0; i14 < loop_ub; i14++) {
    faces->data[i14] = 0.0;
  }

  k0 = 0;
  c_emxInit_boolean_T(&x, 1);
  c_emxInit_real_T(&idx, 1);
  emxInit_int32_T(&ii, 1);
  c_emxInit_boolean_T(&d_vertices, 1);
  c_emxInit_boolean_T(&e_vertices, 1);
  emxInit_int32_T(&b_ii, 1);
  c_emxInit_real_T(&b_idx, 1);
  c_emxInit_real_T(&c_idx, 1);
  while (k0 <= meshXYZ->size[0] - 1) {
    for (j = 0; j < 3; j++) {
      loop_ub = vertices->size[0];
      b_meshXYZ = meshXYZ->data[k0 + meshXYZ->size[0] * meshXYZ->size[1] * j];
      i14 = x->size[0];
      x->size[0] = loop_ub;
      emxEnsureCapacity((emxArray__common *)x, i14, (int32_T)sizeof(boolean_T));
      for (i14 = 0; i14 < loop_ub; i14++) {
        x->data[i14] = (vertices->data[i14] == b_meshXYZ);
      }

      k = 0;
      i14 = ii->size[0];
      ii->size[0] = x->size[0];
      emxEnsureCapacity((emxArray__common *)ii, i14, (int32_T)sizeof(int32_T));
      nb = 1;
      exitg1 = FALSE;
      while ((exitg1 == FALSE) && (nb <= x->size[0])) {
        guard1 = FALSE;
        if (x->data[nb - 1]) {
          k++;
          ii->data[k - 1] = nb;
          if (k >= x->size[0]) {
            exitg1 = TRUE;
          } else {
            guard1 = TRUE;
          }
        } else {
          guard1 = TRUE;
        }

        if (guard1 == TRUE) {
          nb++;
        }
      }

      if (x->size[0] == 1) {
        if (k == 0) {
          i14 = ii->size[0];
          ii->size[0] = 0;
          emxEnsureCapacity((emxArray__common *)ii, i14, (int32_T)sizeof(int32_T));
        }
      } else {
        if (1 > k) {
          loop_ub = 0;
        } else {
          loop_ub = k;
        }

        i14 = b_ii->size[0];
        b_ii->size[0] = loop_ub;
        emxEnsureCapacity((emxArray__common *)b_ii, i14, (int32_T)sizeof(int32_T));
        for (i14 = 0; i14 < loop_ub; i14++) {
          b_ii->data[i14] = ii->data[i14];
        }

        i14 = ii->size[0];
        ii->size[0] = b_ii->size[0];
        emxEnsureCapacity((emxArray__common *)ii, i14, (int32_T)sizeof(int32_T));
        loop_ub = b_ii->size[0];
        for (i14 = 0; i14 < loop_ub; i14++) {
          ii->data[i14] = b_ii->data[i14];
        }
      }

      i14 = idx->size[0];
      idx->size[0] = ii->size[0];
      emxEnsureCapacity((emxArray__common *)idx, i14, (int32_T)sizeof(real_T));
      loop_ub = ii->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        idx->data[i14] = ii->data[i14];
      }

      b_meshXYZ = meshXYZ->data[(k0 + meshXYZ->size[0]) + meshXYZ->size[0] *
        meshXYZ->size[1] * j];
      i14 = e_vertices->size[0];
      e_vertices->size[0] = idx->size[0];
      emxEnsureCapacity((emxArray__common *)e_vertices, i14, (int32_T)sizeof
                        (boolean_T));
      loop_ub = idx->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        e_vertices->data[i14] = (vertices->data[((int32_T)idx->data[i14] +
          vertices->size[0]) - 1] == b_meshXYZ);
      }

      eml_li_find(e_vertices, ii);
      i14 = b_idx->size[0];
      b_idx->size[0] = ii->size[0];
      emxEnsureCapacity((emxArray__common *)b_idx, i14, (int32_T)sizeof(real_T));
      loop_ub = ii->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        b_idx->data[i14] = idx->data[ii->data[i14] - 1];
      }

      i14 = idx->size[0];
      idx->size[0] = b_idx->size[0];
      emxEnsureCapacity((emxArray__common *)idx, i14, (int32_T)sizeof(real_T));
      loop_ub = b_idx->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        idx->data[i14] = b_idx->data[i14];
      }

      b_meshXYZ = meshXYZ->data[(k0 + (meshXYZ->size[0] << 1)) + meshXYZ->size[0]
        * meshXYZ->size[1] * j];
      i14 = d_vertices->size[0];
      d_vertices->size[0] = idx->size[0];
      emxEnsureCapacity((emxArray__common *)d_vertices, i14, (int32_T)sizeof
                        (boolean_T));
      loop_ub = idx->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        d_vertices->data[i14] = (vertices->data[((int32_T)idx->data[i14] +
          (vertices->size[0] << 1)) - 1] == b_meshXYZ);
      }

      eml_li_find(d_vertices, ii);
      i14 = c_idx->size[0];
      c_idx->size[0] = ii->size[0];
      emxEnsureCapacity((emxArray__common *)c_idx, i14, (int32_T)sizeof(real_T));
      loop_ub = ii->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        c_idx->data[i14] = idx->data[ii->data[i14] - 1];
      }

      i14 = idx->size[0];
      idx->size[0] = c_idx->size[0];
      emxEnsureCapacity((emxArray__common *)idx, i14, (int32_T)sizeof(real_T));
      loop_ub = c_idx->size[0];
      for (i14 = 0; i14 < loop_ub; i14++) {
        idx->data[i14] = c_idx->data[i14];
      }

      faces->data[k0 + faces->size[0] * j] = idx->data[0];
    }

    k0++;
  }

  emxFree_real_T(&c_idx);
  emxFree_real_T(&b_idx);
  emxFree_int32_T(&b_ii);
  emxFree_boolean_T(&e_vertices);
  emxFree_boolean_T(&d_vertices);
  emxFree_int32_T(&ii);
  emxFree_real_T(&idx);
  emxFree_boolean_T(&x);
}

/* End of code generation (retrieve_meshformat.cpp) */
