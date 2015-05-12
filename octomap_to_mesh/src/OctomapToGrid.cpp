/* Include files */
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OcTree.h>

#include <octomap_to_mesh/OctomapToGrid.h>
#include <octomap_to_mesh/rt_nonfinite.h>

#include <octomap_to_mesh/rtwtypes.h>
#include <octomap_to_mesh/OctomapToMesh_types.h>
#include <octomap_to_mesh/OctomapToMesh_struct.h>

bool OctomapToGrid (octomap::OcTree *octomap, OctomapToMesh_T * data) {
  std::string ns = ros::this_node::getName();
  ROS_INFO("ns = %s", ns.c_str());
  double xmin, xmax, ymin, ymax, zmin, zmax;
  if(!ros::param::get (ns+"/bbx/minX", xmin)) {
    ROS_ERROR("Parameter minX has not been set!");
    return false;
  }
  if(!ros::param::get (ns+"/bbx/maxX", xmax)) {
    ROS_ERROR("Parameter maxX has not been set!");
    return false;
  }
  if(!ros::param::get (ns+"/bbx/minY", ymin)) {
    ROS_ERROR("Parameter minY has not been set!");
    return false;
  }
  if(!ros::param::get (ns+"/bbx/maxY", ymax)) {
    ROS_ERROR("Parameter maxY has not been set!");
    return false;
  }
  if(!ros::param::get (ns+"/bbx/minZ", zmin)) {
    ROS_ERROR("Parameter minZ has not been set!");
    return false;
  }
  if(!ros::param::get (ns+"/bbx/maxZ", zmax)) {
    ROS_ERROR("Parameter maxZ has not been set!");
    return false;
  }
  
  ROS_INFO("min: (%2.2f, %2.2f, %2.2f) max: (%2.2f, %2.2f, %2.2f)", xmin, ymin, zmin, xmax, ymax, zmax);
  
  // octomap::OcTreeNode * origin = octomap->search (xmin, ymin, zmin);
  octomap::point3d originPoint (xmin, ymin, zmin);
  octomap::OcTreeKey originKey = octomap->coordToKey (originPoint);
  originPoint = octomap->keyToCoord (originKey);
  
  data->octomap_gridX_size[0] = 1;
  data->octomap_gridY_size[0] = 1;
  data->octomap_gridZ_size[0] = 1;
  
  data->octomap_gridX_size[1] = floor ((xmax - xmin) / octomap->getResolution ());
  data->octomap_gridY_size[1] = floor ((ymax - ymin) / octomap->getResolution ());
  data->octomap_gridZ_size[1] = floor ((zmax - zmin) / octomap->getResolution ());
  
  data->octomap_gridX_data = new double[data->octomap_gridX_size[1]];
  data->octomap_gridY_data = new double[data->octomap_gridY_size[1]];
  data->octomap_gridZ_data = new double[data->octomap_gridZ_size[1]];
  
  // TODO(birchera): set min values to center of node 'origin'
  for (int ix = 0; ix < data->octomap_gridX_size[1]; ix++)
    data->octomap_gridX_data[ix] = originPoint.x() + ix * octomap->getResolution();
    
  for (int iy = 0; iy < data->octomap_gridY_size[1]; iy++)
    data->octomap_gridY_data[iy] = originPoint.y() + iy * octomap->getResolution();
    
  for (int iz = 0; iz < data->octomap_gridZ_size[1]; iz++)
    data->octomap_gridZ_data[iz] = originPoint.z() + iz * octomap->getResolution();
  
  /*
  struct emxArray_real_T
  {
      real_T *data;
      int32_T *size;
      int32_T allocatedSize;
      int32_T numDimensions;
      boolean_T canFreeData;
  };
  */

  data->octomap_voxels_map = new emxArray_real_T;
  data->octomap_voxels_map->numDimensions = 3;
  data->octomap_voxels_map->canFreeData = false;
  data->octomap_voxels_map->size = new int32_T[3];
  data->octomap_voxels_map->size[0] = data->octomap_gridX_size[1];
  data->octomap_voxels_map->size[1] = data->octomap_gridY_size[1];
  data->octomap_voxels_map->size[2] = data->octomap_gridZ_size[1];
  data->octomap_voxels_map->data = new real_T[data->octomap_voxels_map->size[0] *
                                              data->octomap_voxels_map->size[1] *
                                              data->octomap_voxels_map->size[2]];
  data->octomap_voxels_map->allocatedSize = sizeof(real_T) * data->octomap_voxels_map->size[0] *
                                                             data->octomap_voxels_map->size[1] *
                                                             data->octomap_voxels_map->size[2];

  for (int ix = 0; ix < data->octomap_gridX_size[1]; ix++) {
    
    for (int iy = 0; iy < data->octomap_gridY_size[1]; iy++) {
    
      for (int iz = 0; iz < data->octomap_gridZ_size[1]; iz++) {
        // TODO(birchera): set min values from origin node center
        octomap::OcTreeNode * node = octomap->search (
                                      originPoint.x() + ix * octomap->getResolution(),
                                      originPoint.y() + iy * octomap->getResolution(),
                                      originPoint.z() + iz * octomap->getResolution());
        // set to occupied unless free
        int index = ix +
                    iy * data->octomap_voxels_map->size[0] +
                    iz * data->octomap_voxels_map->size[0] * data->octomap_voxels_map->size[1];
        if(node == NULL) {
          data->octomap_voxels_map->data[index] = 1;
        }
        else if(octomap->isNodeOccupied(node)) {
          data->octomap_voxels_map->data[index] = 1;
        }
        else {
          data->octomap_voxels_map->data[index] = 0;
        }
      
      }
    }
  }
  
  return true; 
}
