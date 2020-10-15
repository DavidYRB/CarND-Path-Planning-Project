#ifndef MIO
#define MIO

#include <Eigen/Dense>

#include "vehicle.h"

class Mio : public Vehicle{
  private:
    double vel_x;
    double vel_y;
    double pred_start_s;
    double pred_start_d;

  public:
    void updatePredStart();
    Eigen::Vector2d getPredStart();
    Eigen::Vector2d getCartesionVel();
};

#endif