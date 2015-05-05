#ifndef __OCTOMAPTOMESH_STRUCT_H__
#define __OCTOMAPTOMESH_STRUCT_H__

struct OctomapToMesh_T {

  real_T * octomap_gridX_data;
  int32_T octomap_gridX_size[2];
  real_T * octomap_gridY_data;
  int32_T octomap_gridY_size[2];
  real_T * octomap_gridZ_data;
  int32_T octomap_gridZ_size[2];
  
  emxArray_real_T * octomap_voxels_map;
};

#endif // __OCTOMAPTOMESH_STRUCT_H__
