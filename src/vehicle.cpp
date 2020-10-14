#include "vehicle.h"

Vehicle::Vehicle(){
    x = 0.0;
    y = 0.0;
    s = 0.0;
    d = 0.0;
    yaw = 0.0;
    vel = 0.0;
}

void Vehicle::initMIOs(){
    mios[MIOPoses::Front] = nullptr;
    mios[MIOPoses::FrontLeft] = nullptr;
    mios[MIOPoses::FrontRight] = nullptr;
    mios[MIOPoses::RearLeft] = nullptr;
    mios[MIOPoses::RearRight] = nullptr;
}