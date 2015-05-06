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
  
  ros::Duration(1.0).sleep();
  
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
      char_T * fileName = new char_T[50];
      strcpy(fileName, (pkgPath + "/data/" + meshFile).c_str());
      OctomapToMesh(&data, fileName, MeshedOctomap);
    }
  }
  else {
  
    ROS_ERROR("Failed to call service octomap to mesh conversion");
    return 1;
  }

  return 0;
}
