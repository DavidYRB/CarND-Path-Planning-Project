#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          double ref_v = 49.5;
          double dist_c = 0.4;
          int lane_num = 0;
          int traj_pts_num = 50;

          vector<double> ptsx;
          vector<double> ptsy;

          int prev_size = previous_path_x.size();
          double anchor_x = car_x;
          double anchor_y = car_y;
          double anchor_theta = deg2rad(car_yaw);

          if(prev_size < 2){
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
          }
          else{
            anchor_x = previous_path_x[prev_size-1];
            anchor_y = previous_path_y[prev_size-1];
            
            double prev_x = previous_path_x[prev_size-2];
            double prev_y = previous_path_y[prev_size-2];

            ptsx.push_back(prev_x);
            ptsx.push_back(anchor_x);
            ptsy.push_back(prev_y);
            ptsy.push_back(anchor_y);
            
            anchor_theta = atan2(anchor_y - prev_y, anchor_x - prev_x);
          }

          
          vector<int> dist_interval = {30, 60, 90};
          vector<double> wp1 = getXY(car_s + dist_interval[0], (2+4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + dist_interval[1], (2+4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp3 = getXY(car_s + dist_interval[2], (2+4*lane_num), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(wp1[0]);
          ptsx.push_back(wp2[0]);
          ptsx.push_back(wp3[0]);
          ptsy.push_back(wp1[1]);
          ptsy.push_back(wp2[1]);
          ptsy.push_back(wp3[1]);

          for(int i = 0; i < ptsx.size(); ++i){
            double shift_x = ptsx[i] - anchor_x;
            double shift_y = ptsy[i] - anchor_y;
            ptsx[i] = shift_x*cos(0 - anchor_theta) - shift_y*sin(0 - anchor_theta);
            ptsy[i] = shift_x*sin(0 - anchor_theta) + shift_y*cos(0 - anchor_theta);
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          for(int i = 0; i < prev_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
          double increment_dist = target_x / (target_dist/(0.02 * ref_v / 2.24));
          double temp_x = 0;
          double temp_y = 0;
          double global_x;
          double global_y;
          for(int i = 1; i <= traj_pts_num - prev_size; ++i){
            temp_x += increment_dist;
            temp_y = s(temp_x);

            global_x = temp_x*cos(anchor_theta) - temp_y*sin(anchor_theta);
            global_y = temp_x*sin(anchor_theta) + temp_y*cos(anchor_theta);
            global_x += anchor_x;
            global_y += anchor_y;

            next_x_vals.push_back(global_x);
            next_y_vals.push_back(global_y);
          }
          //std::cout << "trajectory size: " << next_x_vals.size() << std::endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}