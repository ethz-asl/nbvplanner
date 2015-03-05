#include "nbvPlanner/nbvp.h"
#include <cstdlib>

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::nbvPlanner()
{

}

template<typename stateVec>
nbvInspection::nbvPlanner<stateVec>::~nbvPlanner()
{

}

template<typename stateVec>
std::vector<stateVec> nbvInspection::nbvPlanner<stateVec>::expand(nbvPlanner<stateVec>& instance, int N, int M, stateVec s, stateVec (nbvInspection::nbvPlanner<stateVec>::*sample)(stateVec), double (nbvInspection::nbvPlanner<stateVec>::*informationGain)(stateVec))
{
  double IG = (instance.*informationGain)(s);
  std::vector<stateVec> path;
  std::vector<stateVec> ret;
  ret.push_back(s);
  
  if(N<=0)
    return ret;
    
  for(int m = 0; m<M; m++)
  {
    path = instance.expand(instance, N-1, M, (instance.*sample)(s), sample, informationGain);
    double IGtmp = (instance.*informationGain)(path.front());
    if(IGtmp>IG)
    {
      ret = path;
      IG = IGtmp;
    }
  }
  
  return ret;
}

template<typename stateVec>
stateVec nbvInspection::nbvPlanner<stateVec>::sampleHolonomic(stateVec s)
{
  stateVec extension;
  for(int i = 0; i<extension.size(); i++)
    extension[i] = EXTENSION_RANGE*(((double)rand())/((double)RAND_MAX)-0.5);
  return s+extension; // TODO: add collision check
}

template<typename stateVec>
double nbvInspection::nbvPlanner<stateVec>::informationGainSimple(stateVec s)
{
  return ((double)rand())/((double)RAND_MAX);
}

