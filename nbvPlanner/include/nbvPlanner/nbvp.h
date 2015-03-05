#include <vector>
#define EXTENSION_RANGE 0.5
#define SQ(x) (x*x)

namespace nbvInspection
{
  template<typename stateVec>
  class nbvPlanner
  {
  public:
    nbvPlanner();
    ~nbvPlanner();
    std::vector<stateVec> expand(nbvPlanner<stateVec>& instance, int N, int M, stateVec s, stateVec (nbvPlanner<stateVec>::*sample)(stateVec), double (nbvPlanner<stateVec>::*informationGain)(stateVec));
    stateVec sampleHolonomic(stateVec s);
    double informationGainSimple(stateVec s);
    // octomap octomap; 
  };
}
