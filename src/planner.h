#ifndef PLANNER
#define PLANNER

#include <string>
#include <vector>
#include <Eigen/Dense>

#include "utils.hpp"
class Planner{
  private:
    Eigen::Matrix<double, 2, 6> best_trajectory;
    States best_state;
    double cost_min;

  public:
    Planner();
    void reset();
    Eigen::VectorXd generateTrajectory(Eigen::Vector3d& init, Eigen::Vector3d& final, double T);
    double trajectoryCost(Eigen::VectorXd& traj, double T);
    void getBestTrajectory(std::vector<States>& available_states);
};

#endif