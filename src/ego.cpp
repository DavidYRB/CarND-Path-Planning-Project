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