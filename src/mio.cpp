#include "mio.h"

Eigen::Vector2d Mio::getCartesionVel(){
    return Eigen::Vector2d(vel_x, vel_y);
}

Eigen::Vector2d Mio::getPredStart(){
    return Eigen::Vector2d(pred_start_s, pred_start_d);
}