#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include "octomap/octomap.h"
#include "octomap/OcTreeNode.h"
#include "octomap/OcTree.h"

#define SQ(x) ((x)*(x))
#define SQRT2 0.70711

namespace nbvInspection
{
  
  template<typename stateVec>
  class Node
  {
    static int counter;
  public:
    Node();
    ~Node();
    Node * minDist(stateVec);
    static int getCounter();
    void printToFile(std::fstream& file);

    stateVec state;
    Node * parent;
    std::vector<Node*> children;
    double informationGain;
    static double bestInformationGain;
    static Node * bestNode;
    static const double ZERO_INFORMATION_GAIN;
  };

  template<typename stateVec>
  class nbvPlanner
  {
    static double v_max;
    static double dyaw_max;
    static double dv_max;
    static double ddyaw_max;
    static double camPitch;
    static double camHorizontal;
    static double camVertical;
    
    static double igFree;
    static double igOccupied;
    static double igUnmapped;
    static double informationGainRange;
    static double degressiveCoeff;
    static double extensionRange;
    static int initIterations;
    static double dt;
    static bool RRTextension;
    
    static double minX;
    static double minY;
    static double minZ;
    static double maxX;
    static double maxY;
    static double maxZ;
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
    octomap_t * octomap;
    std::vector<Eigen::Vector3f> camBoundNormals;
    Node<stateVec> * rootNode;
  };
}
