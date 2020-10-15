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
    
        
  public:
    Vehicle();
    Eigen::VectorXd getStatus();
    void updateStatus(double sensor_x, double sensor_y, double sensor_s, double sensor_d, double sensor_yaw, double sensor_vel);
};

#endif