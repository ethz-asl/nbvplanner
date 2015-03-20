#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "octomap/octomap.h"
#include "octomap/OcTreeNode.h"
#include "octomap/OcTree.h"
#define EXTENSION_RANGE 0.5
#define VMAX 0.25
#define YAWMAX 0.5
#define SQ(x) (x*x)
#define PITCH 0.0
#define CAM_HORIZONTAL 120.0
#define CAM_VERTICAL 120.0
#define SQRT2 0.70711

namespace nbvInspection
{
  template<typename stateVec>
  class nbvplanner
  {
  public:
    typedef std::vector<stateVec> vector_t;
    typedef octomap::OcTree octomap_t;
    
    nbvplanner();
    ~nbvplanner();
    vector_t expand(nbvplanner<stateVec>& instance, int N, int M, vector_t s, vector_t (nbvplanner<stateVec>::*sample)(stateVec), double (nbvplanner<stateVec>::*informationGain)(stateVec));
    vector_t sampleHolonomic(stateVec s);
    vector_t sampleEuler(stateVec s);
    double informationGainRand(stateVec s);
    double informationGainSimple(stateVec s);
    double informationGainCone(stateVec s);
    octomap_t * octomap;
    std::vector<Eigen::Vector3f> camBoundNormals;
  };
}
