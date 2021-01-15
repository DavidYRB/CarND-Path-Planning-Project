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
    double vel;
        
  public:
    Vehicle();
    Vehicle(double sensor_x, double sensor_y, double sensor_vel, double pred_s_init, double pred_d_init);
    Eigen::VectorXd getStatus();
    virtual void updateStatus(double sensor_x, double sensor_y, double sensor_vel, double pred_s_init, double pred_d_init);
};

#endif