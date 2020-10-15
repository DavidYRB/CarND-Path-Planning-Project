#include "vehicle.h"

class Ego : public Vehicle{
  private:
    double yaw;
    double vel;
    
    // most important objects to ego, empty for other vehicles
    std::unordered_map<MIOPoses, std::shared_ptr<Vehicle>> mios;
    // best state before new trajectory being added
    States next_best_state;
    // best trajectory coeffecients for ego
    Eigen::Matrix<double, 2, 6> best_traj_coeff;
    // Planner objects for current vehicle
    Planner path_planner;

    void initMIOs();

  public:
    void resetPlanner();
    void resetMIOS();
    void updateVehicleStatus(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
    void updateMIOS(std::vector<std::vector<double>>& sensor_fusion);
    std::vector<States> getAvailableStates();
    void getBestTrajectory(std::vector<States>& available_states);
}