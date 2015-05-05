octomap_to_mesh
===============

This package supplies functions to generate a surface mesh from an octomap::OcTree. The use this package, add the following c++ snipplet:


```cpp
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <rt_nonfinite.h>
#include <rtwtypes.h>
#include <OctomapToMesh_types.h>

...

emxArray_real_T * octomap_voxels_map = NULL; 
real_T *octomap_gridX_data = NULL; 
int32_T *octomap_gridX_size = NULL; 
real_T *octomap_gridY_data = NULL; 
int32_T *octomap_gridY_size = NULL; 
real_T *octomap_gridZ_data = NULL; 
int32_T *octomap_gridZ_size = NULL; 

if(OctomapToGrid (octomap, octomap_voxels_map,
                    octomap_gridX_data, octomap_gridX_size,
                    octomap_gridY_data, octomap_gridY_size,
                    octomap_gridZ_data, octomap_gridZ_size)) {
  
  struct_T * MeshedOctomap = NULL;
  int strLenght = 11;
  char_T * fileName = new char_T[strLenght];
  *fileName = "meshOut.stl";
  OctomapToMesh(octomap_voxels_map, octomap_gridX_data, octomap_gridX_size,
                                    octomap_gridY_data, octomap_gridY_size,
                                    octomap_gridZ_data, octomap_gridZ_size,
                                    fileName, strLenght, MeshedOctomap);
}

...

```
