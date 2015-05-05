/*
 * circshift.cpp
 *
 * Code generation for function 'circshift'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/circshift.h"
#include "octomap_to_mesh/ind2sub.h"
#include "octomap_to_mesh/OctomapToMesh_rtwutil.h"
#include <stdio.h>
#include <algorithm>

/* Function Definitions */
void b_circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i24;
  static const int8_T iv2[3] = { 1, 0, 0 };

  int32_T rm1;
  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[1] == 0) || (a->size[2] == 0) || ((a->size[0] == 1) && (a->size[1]
        == 1) && (a->size[2] == 1))) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i24 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        rm1 = iv2[dim] - div_s32((int32_T)iv2[dim], a->size[dim]) * a->size[dim];
        if (rm1 > 0) {
          if (iv2[dim] > 0) {
            rm1 = a->size[dim] - 1;
          }

          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i24; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = rm1; k + 1 <= i24; k++) {
                a->data[ia] = atmp_data[k];
                ia += vstride;
              }

              for (k = 1; k <= rm1; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }
            }
          }
        }
      }

      vstride *= i24;
    }
  }
  delete[] atmp_data;
}

void c_circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i25;
  static const int8_T iv3[3] = { 0, -1, 0 };

  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[0] == 0) || (a->size[2] == 0) || ((a->size[0] == 1) && (a->size[1]
        == 1) && (a->size[2] == 1))) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i25 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        k = (int32_T)fabs((real_T)iv3[dim]);
        if (k - div_s32(k, a->size[dim]) * a->size[dim] > 0) {
          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i25; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = 2; k <= i25; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }

              a->data[ia] = atmp_data[0];
            }
          }
        }
      }

      vstride *= i25;
    }
  }
  delete[] atmp_data;
}

void circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i23;
  static const int8_T iv1[3] = { -1, 0, 0 };

  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[1] == 0) || (a->size[2] == 0) || ((a->size[0] == 1) && (a->size[1]
        == 1) && (a->size[2] == 1))) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i23 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        k = (int32_T)fabs((real_T)iv1[dim]);
        if (k - div_s32(k, a->size[dim]) * a->size[dim] > 0) {
          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i23; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = 2; k <= i23; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }

              a->data[ia] = atmp_data[0];
            }
          }
        }
      }

      vstride *= i23;
    }
  }
  delete[] atmp_data;
}

void d_circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i26;
  static const int8_T iv4[3] = { 0, 1, 0 };

  int32_T rm1;
  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[0] == 0) || (a->size[2] == 0) || ((a->size[0] == 1) && (a->size[1]
        == 1) && (a->size[2] == 1))) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i26 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        rm1 = iv4[dim] - div_s32((int32_T)iv4[dim], a->size[dim]) * a->size[dim];
        if (rm1 > 0) {
          if (iv4[dim] > 0) {
            rm1 = a->size[dim] - 1;
          }

          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i26; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = rm1; k + 1 <= i26; k++) {
                a->data[ia] = atmp_data[k];
                ia += vstride;
              }

              for (k = 1; k <= rm1; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }
            }
          }
        }
      }

      vstride *= i26;
    }
  }
  delete[] atmp_data;
}

void e_circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i27;
  static const int8_T iv5[3] = { 0, 0, -1 };

  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[0] == 0) || (a->size[1] == 0)) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i27 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        k = (int32_T)fabs((real_T)iv5[dim]);
        if (k - div_s32(k, a->size[dim]) * a->size[dim] > 0) {
          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i27; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = 2; k <= i27; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }

              a->data[ia] = atmp_data[0];
            }
          }
        }
      }

      vstride *= i27;
    }
  }
  delete[] atmp_data;
}

void f_circshift(emxArray_real_T *a)
{
  real_T * atmp_data = new real_T[std::max(a->size[0],std::max(a->size[1],a->size[2]))];
  int32_T vstride;
  int32_T npages;
  int32_T k;
  real_T u0;
  int32_T minval;
  int32_T dim;
  int32_T i28;
  static const int8_T iv6[3] = { 0, 0, 1 };

  int32_T rm1;
  int32_T vspread;
  int32_T i2;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T ia;
  if ((a->size[0] == 0) || (a->size[1] == 0)) {
  } else {
    vstride = 1;
    npages = a->size[0] * a->size[1] * a->size[2];
    k = 3;
    while ((k > 2) && (a->size[2] == 1)) {
      k = 2;
    }

    u0 = k;
    minval = (int32_T)u0;
    for (dim = 0; dim + 1 <= minval; dim++) {
      i28 = a->size[dim];
      npages = div_s32(npages, a->size[dim]);
      if (a->size[dim] > 1) {
        rm1 = iv6[dim] - div_s32((int32_T)iv6[dim], a->size[dim]) * a->size[dim];
        if (rm1 > 0) {
          if (iv6[dim] > 0) {
            rm1 = a->size[dim] - 1;
          }

          vspread = (a->size[dim] - 1) * vstride;
          i2 = 0;
          for (i = 1; i <= npages; i++) {
            i1 = i2;
            i2 += vspread;
            for (j = 1; j <= vstride; j++) {
              i1++;
              i2++;
              ia = i1;
              for (k = 1; k <= i28; k++) {
                atmp_data[k - 1] = a->data[ia - 1];
                ia += vstride;
              }

              ia = i1 - 1;
              for (k = rm1; k + 1 <= i28; k++) {
                a->data[ia] = atmp_data[k];
                ia += vstride;
              }

              for (k = 1; k <= rm1; k++) {
                a->data[ia] = atmp_data[k - 1];
                ia += vstride;
              }
            }
          }
        }
      }

      vstride *= i28;
    }
  }
  delete[] atmp_data;
}

/* End of code generation (circshift.cpp) */
