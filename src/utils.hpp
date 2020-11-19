#ifndef MY_UTILS
#define MY_UTILS

#define MAX_COST 99999
#define TIME_INTERVAL 0.1
#define TRAJECTORY_TIME_INTERVAL 0.02
#include <cfloat>
#include <Eigen/Dense>
#include <math.h>
#include <vector>


enum class State{KeepLane, PreLaneChangeLeft, PreLaneChangeRight, LaneChangeLeft, LaneChangeRight};
enum class MIOPoses{FrontLeft, FrontRight, Front, RearLeft, RearRight};


double polynominal(const Eigen::VectorXd& coeff, const double x){
    double result{0.0};
    for(int i = 0; i < coeff.size(); ++i){
        result += coeff(i) * pow(x, i);
    }
    return result;
}
#endif 

int getLaneNumber(const double d){
    if(d >= 0 && d <= 4){
        return 0;
    }
    else if(d >4 && d <= 8){
        return 1;
    }
    else{
        return 2;
    }

}

double getCartesionVel(double x_vel, double y_vel){
    sqrt(pow(x_vel, 2) + pow(y_vel, 2));
}

// TODO: Backbone functions
std::vector<double> getTrajCoeff(Eigen::Vector3d init_cond, Eigen::Vector3d final_cond, double horizon_t){}

std::vector<State> getAvailableStates(State curr_state, int curr_d){}

std::vector<std::vector<double>> generateFinalContidionsKL(Eigen::Vector3d s_init, Eigen::Vector3d d_init, double horizon_t){}
std::vector<std::vector<double>> generateFinalContidionsPLC(Eigen::Vector3d s_init, Eigen::Vector3d d_init, double horizon_t){}
std::vector<std::vector<double>> generateFinalContidionsLC(Eigen::Vector3d s_init, Eigen::Vector3d d_init, double horizon_t, int target_lane){}

double calculateCost(const std::vector<double>& s_traj_coeff, const std::vector<double>& d_traj_coeff, 
                     const Eigen::Vector3d& s_init, const Eigen::Vector3d& d_init, double horizon_t)
{

}

// cost functions
