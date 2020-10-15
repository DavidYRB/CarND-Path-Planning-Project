#include <math.h>

#include "mio.h"
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

double Planner::trajectoryCost(Eigen::VectorXd& traj, double T, Eigen::Vector3d& init, std::vector<std::shared_ptr<Mio>>& mios){
    // we need mios info here, we need

}

double Planner::nearestDist(const Eigen::VectorXd& s_traj, const Eigen::VectorXd& d_traj, const double T, const std::shared_ptr<Mio> mio){
    Eigen::VectorXd mio_status = mio->getStatus();
    Eigen::Vector2d mio_pred_start = mio->getPredStart();
    Eigen::Vector2d cartesion_vel = mio->getCartesionVel();
    double vel = sqrt(pow(cartesion_vel(0), 2) + pow(cartesion_vel(1), 2));
    double nearest_dist{DBL_MAX};
    double current_dist{DBL_MAX};

    double ego_s{0.0};
    double ego_d{0.0};
    
    for(int i = 1; i <= 100; ++i){
        double t = i * T / 100.0;
        ego_s = polynominal(s_traj, t);
        ego_d = polynominal(d_traj, t);
        current_dist = sqrt(pow((ego_s - mio_pred_start(0)), 2) + pow((ego_d - mio_pred_start(1)), 2));
        if(current_dist < nearest_dist){
            nearest_dist = current_dist;
        }
    }
    return nearest_dist;
}