#include "vehicle.h"

#include <memory>
#include <unordered_map>
class Ego : public Vehicle{
  private:
    double yaw;
    double vel;
    int remaining_traj_lenth;
    int pred_start_index;

    double end_path_s;
    double end_path_d;

    std::vector<State> state_traj;
    std::vector<double> s_traj;
    std::vector<double> s_dot_traj;
    std::vector<double> s_dot_dot_traj;
    std::vector<double> d_traj;
    std::vector<double> d_dot_traj;
    std::vector<double> d_dot_dot_traj;

    std::vector<double> prev_traj_s;
    std::vector<double> prev_traj_d;
    std::vector<double> curr_traj_s;
    std::vector<double> curr_traj_d;
    double cost;
    double horizon_ref;

    Eigen::Vector3d s_init;
    Eigen::Vector3d d_init;
    State state_init;

    Eigen::Matrix3d s_traj;
    Eigen::Matrix3d d_traj;


    // most important objects to ego, empty for other vehicles
    std::unordered_map<int, std::shared_ptr<Vehicle>> detected_cars;

    bool init_finished;

    void updatePredInit(int remain_size);

  public:
    // initialize ego
    void init();
    // check if ego get initialized
    bool isInitialized();
    // based on sensor data update the ego status (is it necessary?)
    void updateEgoStatus(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, int remain_size);
    // based on sensor fusion data, generate the surronuding car with in range
    void updateOtherVehs(auto sensor_fusion);
    // get the best trajectory and corresponding state.
    void generateBestTrajAndState();
    // fill the points to reach desired trajectory length 
    void updateTrajectory();
    // check if the selected prediction initial equals to the input information
    bool predInitMatches(double end_path_s, double end_path_d);
