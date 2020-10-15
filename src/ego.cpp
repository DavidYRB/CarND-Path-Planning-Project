#include "utils.hpp"
#include "ego.h"

Ego::Ego(){
    initMIOs();
    best_traj_coeff = Eigen::MatrixXd::Zero(2,6);
}

void Ego::initMIOs(){
    mios[MIOPoses::Front] = nullptr;
    mios[MIOPoses::FrontLeft] = nullptr;
    mios[MIOPoses::FrontRight] = nullptr;
    mios[MIOPoses::RearLeft] = nullptr;
    mios[MIOPoses::RearRight] = nullptr;
}

void Ego::getBestTrajectory(std::vector<States>& available_states){
    double cost{DBL_MAX};
    for(auto state : available_states){
        // 1. generate final state vectors
        // 2. get the vector of MIOs
        // 3. for each final state:
        //      a. calculate the trajectory coefficients (2*6)
        //      b. for each MIO:
        //          i. calculate nearest distant in the prediction horizon between MIO and ego
        //          ii. calculate the cost
        //          iii. add cost together
        //      c. calculate the average cost for this trajectory, update cost if the new one is lower
    }
}