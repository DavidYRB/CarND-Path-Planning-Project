#ifndef PLANNER
#define PLANNER

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "mio.h"
#include "utils.hpp"
class Planner{
  private:
    Eigen::Matrix<double, 2, 6> best_trajectory;
    States best_state;
    double cost_min;

    double collisionCost(const Eigen::VectorXd& traj, const double T, const Eigen::Vector3d& init, const std::shared_ptr<Vehicle> mio);
    double bufferCost();
    double effecientCost();
    double totalAcclCost();
    double totalJerkCost();
    double maxAcclCost();
    double maxJerkCost();
    double nearestDist(const Eigen::VectorXd& s_traj, const Eigen::VectorXd& d_traj, const double T, const std::shared_ptr<Mio> mio);

  public:
    Planner();
    void reset();
    Eigen::VectorXd generateTrajectory(const Eigen::Vector3d& init, const Eigen::Vector3d& final, const double T);
    double trajectoryCost(const Eigen::VectorXd& traj, const double T, const Eigen::Vector3d& init, const std::vector<std::shared_ptr<Vehicle>>& mios);
    void getBestTrajectory(const std::vector<States>& available_states);
};

#endif