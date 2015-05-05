/*
 * fopen.cpp
 *
 * Code generation for function 'fopen'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/fopen.h"
#include "octomap_to_mesh/fileManager.h"
#include <stdio.h>

/* Function Definitions */
real_T b_fopen(const char_T * filename_data, const int32_T filename_size[2])
{
  return fileManager(filename_data, filename_size);
}

/* End of code generation (fopen.cpp) */
