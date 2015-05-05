/*
 * unique.cpp
 *
 * Code generation for function 'unique'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/unique.h"
#include <stdio.h>

/* Function Definitions */
boolean_T rows_differ(const emxArray_real_T *b, int32_T k0, int32_T k)
{
  boolean_T p;
  int32_T j;
  boolean_T exitg1;
  real_T absxk;
  int32_T exponent;
  boolean_T b_p;
  p = FALSE;
  j = 0;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (j < 3)) {
    absxk = fabs(b->data[(k + b->size[0] * j) - 1] / 2.0);
    if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
      if (absxk <= 2.2250738585072014E-308) {
        absxk = 4.94065645841247E-324;
      } else {
        frexp(absxk, &exponent);
        absxk = ldexp(1.0, exponent - 53);
      }
    } else {
      absxk = rtNaN;
    }

    if ((fabs(b->data[(k + b->size[0] * j) - 1] - b->data[(k0 + b->size[0] * j)
              - 1]) < absxk) || (rtIsInf(b->data[(k0 + b->size[0] * j) - 1]) &&
         rtIsInf(b->data[(k + b->size[0] * j) - 1]) && ((b->data[(k0 + b->size[0]
            * j) - 1] > 0.0) == (b->data[(k + b->size[0] * j) - 1] > 0.0)))) {
      b_p = TRUE;
    } else {
      b_p = FALSE;
    }

    if (!b_p) {
      p = TRUE;
      exitg1 = TRUE;
    } else {
      j++;
    }
  }

  return p;
}

/* End of code generation (unique.cpp) */
