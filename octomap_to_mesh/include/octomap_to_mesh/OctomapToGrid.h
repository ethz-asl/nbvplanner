#ifndef __OCTOMAPMSGTOGRID_H__
#define __OCTOMAPMSGTOGRID_H__
/* Include files */
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "octomap_to_mesh/rt_nonfinite.h"

#include "octomap_to_mesh/rtwtypes.h"
#include "octomap_to_mesh/OctomapToMesh_types.h"
#include <octomap_to_mesh/OctomapToMesh_struct.h>

/* Function Declarations */
extern bool OctomapToGrid (octomap::OcTree *octomap, OctomapToMesh_T * data);
#endif // __OCTOMAPMSGTOGRID_H__
