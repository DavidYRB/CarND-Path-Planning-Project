#include "vehicle.h"

#include <memory>
#include <unordered_map>
class Ego : public Vehicle{
  private:
    double yaw;
    double vel;

    State current_state;
    std::vector<double> prev_traj_s;
    std::vector<double> prev_traj_d;
    std::vector<double> curr_traj_s;
    std::vector<double> curr_traj_d;
    double cost;
    double horizon_ref;

    Eigen::Vector3d s_init;
    Eigen::Vector3d d_init;

    Eigen::Matrix3d s_traj;
    Eigen::Matrix3d d_traj;
    std::vector<double> state_traj;


    // most important objects to ego, empty for other vehicles
    std::unordered_map<int, std::shared_ptr<Vehicle>> detected_cars;

    bool init_finished;

  public:
    void init();
    bool isInitialized();
    void updateEgoStatus(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);
    void updateTrajectory(int used_points, const std::vector<double>& s_traj_coeff, const std::vector<double>& d_traj_coeff, std::vector<double>& next_x_vel, std::vector<double>& next_y_vel){}
}