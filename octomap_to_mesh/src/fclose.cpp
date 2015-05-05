/*
 * fclose.cpp
 *
 * Code generation for function 'fclose'
 *
 * C source code generated on: Mon May 04 11:24:36 2015
 *
 */

/* Include files */
#include "octomap_to_mesh/rt_nonfinite.h"
#include "octomap_to_mesh/OctomapToMesh.h"
#include "octomap_to_mesh/fclose.h"
#include "octomap_to_mesh/fileManager.h"
#include <stdio.h>

/* Function Definitions */
void b_fclose(real_T fileID)
{
  c_fileManager(fileID);
}

/* End of code generation (fclose.cpp) */
