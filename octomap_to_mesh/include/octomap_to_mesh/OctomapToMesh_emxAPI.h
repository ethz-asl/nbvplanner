/*
 * OctomapToMesh_emxAPI.h
 *
 * Code generation for function 'OctomapToMesh_emxAPI'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

#ifndef __OCTOMAPTOMESH_EMXAPI_H__
#define __OCTOMAPTOMESH_EMXAPI_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "octomap_to_mesh/rt_nonfinite.h"

#include "octomap_to_mesh/rtwtypes.h"
#include "octomap_to_mesh/OctomapToMesh_types.h"

/* Function Declarations */
extern emxArray_real_T *emxCreateND_real_T(int32_T numDimensions, int32_T *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(real_T *data, int32_T numDimensions, int32_T *size);
extern emxArray_real_T *emxCreateWrapper_real_T(real_T *data, int32_T rows, int32_T cols);
extern emxArray_real_T *emxCreate_real_T(int32_T rows, int32_T cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
#endif
/* End of code generation (OctomapToMesh_emxAPI.h) */
