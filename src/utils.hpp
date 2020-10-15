#ifndef MY_UTILS
#define MY_UTILS

#define MAX_COST 99999
#define TIME_INTERVAL 0.1
#include <cfloat>
#include <Eigen/Dense>
#include <math.h>
#include <vector>


enum class States{KeepLane, PreLaneChangeLeft, PreLaneChangeRight, LaneChangeLeft, LaneChangeRight};
enum class MIOPoses{FrontLeft, FrontRight, Front, RearLeft, RearRight};


double polynominal(const Eigen::VectorXd& coeff, const double x){
    double result{0.0};
    for(int i = 0; i < coeff.size(); ++i){
        result += coeff(i) * pow(x, i);
    }
    return result;
}
#endif 