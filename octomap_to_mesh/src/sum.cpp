/*
 * sum.cpp
 *
 * Code generation for function 'sum'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/sum.h"
#include <stdio.h>

/* Function Definitions */
real_T b_sum(const boolean_T x[6])
{
  real_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 5; k++) {
    y += (real_T)x[k + 1];
  }

  return y;
}

real_T sum(const emxArray_real_T *x)
{
  real_T y;
  int32_T k;
  if (x->size[1] == 0) {
    y = 0.0;
  } else {
    y = x->data[0];
    for (k = 2; k <= x->size[1]; k++) {
      y += x->data[k - 1];
    }
  }

  return y;
}

/* End of code generation (sum.cpp) */
