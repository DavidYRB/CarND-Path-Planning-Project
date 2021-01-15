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

    s_init(0) = s_traj[pred_start_index];
    s_init(1) = s_dot_traj[pred_start_index];
    s_init(2) = s_dot_dot_traj[pred_start_index];
    d_init(0) = d_traj[pred_start_index]; 
    d_init(1) = d_dot_traj[pred_start_index];
    d_init(2) = d_dot_dot_traj[pred_start_index];

    state_init = state_traj[pred_start_index];
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

void Ego::updateOtherVehs(auto sensor_fusion, int remain_size){
    // for each car, check the car within SENSORRANGE distance for lanes next to current one
    // if it within the range, create a car instance and put it into map with ID as key
    // 
    int size = sensor_fusion.size();
    for(int i = 0; i < size; ++i){
        int id = sensor_fusion[i][0];
        double x = sensor_fusion[i][1];
        double y = sensor_fusion[i][2];
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double vel = sqrt(vx*vx + vy*vy);
        double s = sensor_fusion[i][5] + remain_size * TRAJECTORY_TIME_INTERVAL * vel;
        double d = sensor_fusion[i][6];
        if(abs(s_init[0] - s) < SENSORRANGE){
            if(detected_cars.find(id) != detected_cars.end()){
                detected_cars[id]->updateStatus(x, y, vel, s, d);
            }
            else{
                detected_cars[id] = std::make_shared<Vehicle>(x, y, vel, s, d);
            }
        }
        else{
            if(detected_cars.find(id) != detected_cars.end()){
                detected_cars.erase(id);
            }
        }
    }
}
