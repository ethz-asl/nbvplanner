#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

#include <octomap_to_mesh/rt_nonfinite.h>
#include <octomap_to_mesh/rtwtypes.h>
#include <octomap_to_mesh/OctomapToMesh_types.h>
#include <octomap_to_mesh/OctomapToMesh_struct.h>
#include <octomap_to_mesh/OctomapToGrid.h>
#include <octomap_to_mesh/OctomapToMesh.h>

typedef octomap_msgs::GetOctomap OctomapSrv;

int main(int argc, char **argv) {

  ros::init(argc, argv, "octomap_to_mesh");

  ros::NodeHandle n;
  ros::ServiceClient octomapClient = n.serviceClient<OctomapSrv>("octomap_full");
  std::string pkgPath = ros::package::getPath("octomap_to_mesh");
  
  // Wait a moment, without relying on the clock (for cases use_sim_time = true)
  for (int i = 0; i < 999999; i++);
  OctomapSrv srv;
  
  if (octomapClient.call(srv)) {
  
    OctomapToMesh_T data;
    
    octomap::OcTree * octomap;
    
    if(srv.response.map.binary) {
    
      octomap = octomap_msgs::binaryMsgToMap(srv.response.map);
    }
    else {
    
      octomap::AbstractOcTree * tree = octomap_msgs::fullMsgToMap(srv.response.map);
      octomap = dynamic_cast<octomap::OcTree*>(tree);
    }

    if (OctomapToGrid (octomap, &data)) {
      
      std::string meshFile = "meshOut.stl";
      std::string ns = ros::this_node::getName();
      if(!ros::param::get (ns+"/mesh_file_name", meshFile))
        ROS_WARN("No mesh file name specified. Using default (meshOut.stl)");
  
      struct_T * MeshedOctomap = NULL;
      std::string fullFile = pkgPath + "/data/" + meshFile;
      char_T * fileName = new char_T[(int)fullFile.size()+5];
      strcpy(fileName, fullFile.c_str());
      OctomapToMesh(&data, fileName, MeshedOctomap);
    }
  }
  else {
  
    ROS_ERROR("Failed to call service octomap to mesh conversion");
    return 1;
  }

  ROS_INFO("Successfully converted octomap to mesh");
  return 0;
}
