/*
 * ind2sub.h
 *
 * Code generation for function 'ind2sub'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

#ifndef __IND2SUB_H__
#define __IND2SUB_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "octomap_to_mesh/rt_nonfinite.h"

#include "octomap_to_mesh/rtwtypes.h"
#include "octomap_to_mesh/OctomapToMesh_types.h"

/* Function Declarations */
extern void b_ind2sub(const real_T siz[3], real_T ndx, real_T *varargout_1, real_T *varargout_2, real_T *varargout_3);
extern void ind2sub(const real_T siz[3], const emxArray_real_T *ndx, emxArray_real_T *varargout_1, emxArray_real_T *varargout_2, emxArray_real_T *varargout_3);
#endif
/* End of code generation (ind2sub.h) */
