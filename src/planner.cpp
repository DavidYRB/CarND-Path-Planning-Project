#include <math.h>

#include "planner.h"
#include "utils.hpp"

Planner::Planner(){
    best_trajectory = Eigen::MatrixXd::Zero(2,6);
    cost_min = MAX_COST;
}

void Planner::reset(){
    best_trajectory = Eigen::MatrixXd::Zero(2,6);
    cost_min = MAX_COST;
}

Eigen::VectorXd Planner::generateTrajectory(Eigen::Vector3d& init, Eigen::Vector3d& final, double T){
    Eigen::VectorXd traj_coeff;
    traj_coeff(0, 0) = init(0);
    traj_coeff(0, 1) = init(1);
    traj_coeff(0, 2) = init(2) / 2.0;

    Eigen::Matrix3d timeMatrix;
    timeMatrix << pow(T, 3), std::pow(T, 4), pow(T, 5),
                  3*pow(T, 2), 4*pow(T, 3), 5*pow(T, 4),
                  6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
    Eigen::Matrix3d timeMatrixInv = timeMatrix.inverse();

    Eigen::Vector3d remain_coeff;
    double c_0 = final(0) - (init(0) + init(1) * T + init(2) * pow(T, 2) / 2.0);
    double c_1 = final(1) - (init(1) + init(2) * T);
    double c_2 = final(2) - init(2);
    Eigen::Vector3d constant(c_0, c_1, c_2);

    remain_coeff = timeMatrixInv * constant;
    
    traj_coeff << init(0), init(1), init(2)/2.0, remain_coeff(0), remain_coeff(1), remain_coeff(2);
}

double Planner::trajectoryCost(Eigen::VectorXd& traj, double T){
    
}