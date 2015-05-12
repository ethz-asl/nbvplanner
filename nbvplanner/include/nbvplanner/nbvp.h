#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include "octomap/octomap.h"
#include "octomap/OcTreeNode.h"
#include "octomap/OcTree.h"
#include <octomap_world/octomap_manager.h>

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection
{
  
  template<typename stateVec>
  class Node
  {
    static int counter_;
  public:
    Node();
    ~Node();
    Node * minDist(stateVec);
    static int getCounter();
    void printToFile(std::fstream& file);

    stateVec state_;
    Node * parent_;
    std::vector<Node*> children_;
    double informationGain_;
    static double bestInformationGain_;
    static Node * bestNode_;
    static const double ZERO_INFORMATION_GAIN_;
  };

  template<typename stateVec>
  class nbvPlanner
  {
    static double v_max_;
    static double dyaw_max_;
    static double dv_max_;
    static double ddyaw_max_;
    static double camPitch_;
    static double camHorizontal_;
    static double camVertical_;
    
    static double igProbabilistic_;
    static double igFree_;
    static double igOccupied_;
    static double igUnmapped_;
    static double informationGainRange_;
    static double degressiveCoeff_;
    static double extensionRange_;
    static int initIterations_;
    static double dt_;
    static bool RRTextension_;
    
    static double minX_;
    static double minY_;
    static double minZ_;
    static double maxX_;
    static double maxY_;
    static double maxZ_;
    std::stack<stateVec> history_;
  public:
    typedef std::vector<stateVec> vector_t;
    typedef octomap::OcTree octomap_t;
    
    nbvPlanner();
    ~nbvPlanner();
    vector_t expand(nbvPlanner<stateVec>& instance, int N, int M, vector_t s, double& IGout, vector_t (nbvPlanner<stateVec>::*sample)(stateVec), double (nbvPlanner<stateVec>::*informationGain)(stateVec));
    vector_t expandStructured(nbvPlanner<stateVec>& instance, int I, stateVec s, double& IGout, double (nbvPlanner<stateVec>::*informationGain)(stateVec));
    vector_t sampleHolonomic(stateVec s);
    vector_t sampleEuler(stateVec s);
    double informationGainRand(stateVec s);
    double informationGainSimple(stateVec s);
    double informationGainCone(stateVec s);
    bool castRay(octomath::Vector3 origin, octomath::Vector3 direction, octomath::Vector3& end, bool ignoreUnknownCells, double d);
    static bool setParams();
    static bool getRRTextension();
    static int getInitIterations();
    static bool extensionRangeSet();
    octomap_t * octomap_;
    std::vector<Eigen::Vector3d> camBoundNormals_;
    Node<stateVec> * rootNode_;
    static volumetric_mapping::OctomapManager * manager_;
    static Eigen::Vector3d boundingBox_;
  };
}
