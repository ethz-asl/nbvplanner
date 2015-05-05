/*
 * write_stl.h
 *
 * Code generation for function 'write_stl'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

#ifndef __WRITE_STL_H__
#define __WRITE_STL_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "octomap_to_mesh/rt_nonfinite.h"

#include "octomap_to_mesh/rtwtypes.h"
#include "octomap_to_mesh/OctomapToMesh_types.h"

/* Function Declarations */
extern void write_stl(const emxArray_real_T *vertices, const emxArray_real_T *faces, const char_T fname_stl_data[8], const int32_T fname_stl_size[2]);
#endif
/* End of code generation (write_stl.h) */
