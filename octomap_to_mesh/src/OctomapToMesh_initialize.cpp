/*
 * OctomapToMesh_initialize.cpp
 *
 * Code generation for function 'OctomapToMesh_initialize'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/OctomapToMesh_initialize.h"
#include "octomap_to_mesh/OctomapToMesh_data.h"
#include <stdio.h>

/* Function Definitions */
void OctomapToMesh_initialize()
{
  FILE * a;
  int32_T i;
  rt_InitInfAndNaN(8U);
  a = NULL;
  for (i = 0; i < 20; i++) {
    eml_autoflush[i] = FALSE;
    eml_openfiles[i] = a;
  }
}

/* End of code generation (OctomapToMesh_initialize.cpp) */
