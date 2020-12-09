#include <cassert>
#include "utils.hpp"
#include "ego.h"

Ego::Ego(){
    yaw = 0.0;
    vel = 0.0;
    remaining_traj_lenth = 0;
    init_finished = false;
}

void Ego::init(){
    state_traj.resize(TRAJECTORY_LENGTH, State::KeepLane);
    s_traj.resize(TRAJECTORY_LENGTH, 0.0);
    s_dot_traj.resize(TRAJECTORY_LENGTH, 0.0);
    s_dot_dot_traj.resize(TRAJECTORY_LENGTH, 0.0);
    d_traj.resize(TRAJECTORY_LENGTH, 0.0);
    d_dot_traj.resize(TRAJECTORY_LENGTH, 0.0);
    d_dot_dot_traj..resize(TRAJECTORY_LENGTH, 0.0);

    prev_traj_d.resize(COEFFNUMBER, 0.0);
    prev_traj_s.resize(COEFFNUMBER, 0.0);
    curr_traj_d.resize(COEFFNUMBER, 0.0);
    curr_traj_s.resize(COEFFNUMBER, 0.0);

    cost = MAX_COST;
    horizon_ref = HORIZONREF;

    updatePredInit(TRAJECTORY_LENGTH);

    init_finished = true;
}

bool Ego::isInitialized(){
    return init_finished;
}

void Ego::updatePredInit(int remain_size){
    pred_start_index = TRAJECTORY_LENGTH - remain_size;

    s_init << s_traj[pred_start_index], s_dot_traj[pred_start_index], s_dot_dot_traj[pred_start_index];
    d_init << d_traj[pred_start_index], d_dot_traj[pred_start_index], d_dot_dot_traj[pred_start_index];

}

void Ego::updateEgoStatus(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, int remain_size){
    x = car_x;
    y = car_y;
    s = car_s;
    d = car_d;
    yaw = car_yaw;
    vel = car_speed / MPSTOMPH; // conver from mph to m/s
    
    updatePredInit(remain_size);
}

void Ego::updateOtherVehs(auto sensor_fusion){
    // for each car, check the car within SENSORRANGE distance for lanes next to current one
    // if it within the range, create a car instance and put it into map with ID as key
    // 
}
