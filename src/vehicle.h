#ifndef VEHICLE
#define VEHICLE

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "planner.h"
#include "utils.hpp"

class Vehicle{
  private:
    // current vehicle status
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double vel;
    
        
  public:
    Vehicle();
};

#endif