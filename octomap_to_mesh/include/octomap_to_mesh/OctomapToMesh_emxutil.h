/*
 * OctomapToMesh_emxutil.h
 *
 * Code generation for function 'OctomapToMesh_emxutil'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

#ifndef __OCTOMAPTOMESH_EMXUTIL_H__
#define __OCTOMAPTOMESH_EMXUTIL_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "octomap_to_mesh/rt_nonfinite.h"

#include "octomap_to_mesh/rtwtypes.h"
#include "octomap_to_mesh/OctomapToMesh_types.h"

/* Function Declarations */
extern void b_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int32_T numDimensions);
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
extern void c_emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int32_T numDimensions);
extern void c_emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int32_T oldNumel, int32_T elementSize);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int32_T numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
#endif
/* End of code generation (OctomapToMesh_emxutil.h) */
