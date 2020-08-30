#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h" //Used in trajectory generation

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

  // Setting the initials
  int lane = 1;			//initial lane
  double ref_vel = 0.0; //reference velocity in mph

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
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
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

          // Sensor Fusion Data: a list of all other cars on the same side
          auto sensor_fusion = j[1]["sensor_fusion"];
          

          //TODO: define a path made up of (x,y) points
          //that the car will visit sequentially every .02 seconds
          int prev_size = previous_path_x.size();
          
          //Check the surrounding cars to prevent collision
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          bool car_ahead = false;
          bool too_close = false;  //flag for decelaration
          bool car_left = false;
          bool car_right = false;
          
          // find reference velocity ref_v to use
          for (int i=0; i<sensor_fusion.size(); ++i)
          {
            //Car in the current lane
            float d = sensor_fusion[i][6];
            int otherCar_lane = -1;
            if ((d > 0) && (d < 4))
            {
              otherCar_lane = 0;
            }
            else if ((d > 4) && (d < 8))
            {
              otherCar_lane = 1;
            }
            else if ((d > 8) && (d < 12))
            {
              otherCar_lane = 2;
            }
            if (otherCar_lane < 0)
            {
              continue;
            }

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*0.02*check_speed);

            if (lane == otherCar_lane)
            {
              car_ahead |= (check_car_s>car_s) && ((check_car_s-car_s)<30);
            }
            else if (otherCar_lane - lane == 1)
            {
              car_right |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
            }
            else if (otherCar_lane - lane == -1)
            {
              car_left |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
            }
          }

          //Behavior Prediction
          double  speed_diff = 0;
          if (car_ahead)
          {
            if (!car_left && lane > 0)
            { 
              lane--; //change lane to the left
            }
            else if (!car_right && lane != 2)
            {
              lane++; //change lane to the right
            }
            else
            {
              speed_diff -= 0.224;
            }
            }
            else
            {
              if (lane != 1)
              { 
                if ((lane == 0 && !car_right) || (lane == 2 && !car_left))
                {
                  lane = 1; //return to the center
                }
              }
              if (ref_vel < 49.8 )
              {
                speed_diff += 0.224;
              }
            }

          //Initializing (x, y) waypoints
          vector<double> pts_x, pts_y;

          //Reference x, y points and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // use car states as the starting reference if the previous size is almost empty (size < 2)
          if (prev_size < 2)
          {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            pts_x.push_back(prev_car_x);
            pts_x.push_back(car_x);
            pts_y.push_back(prev_car_y);
            pts_y.push_back(car_y);
          }
          //Get the points from the latest path
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            //Inserting the previous points
            pts_x.push_back(ref_x_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y_prev);
            pts_y.push_back(ref_y);
          }

          //Add 30 meters ahead of the start reference in the FRENET coordinate
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          //For x-points
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          
          //For y-points
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          
          //Shifting the car 
          for (int i=0; i<pts_x.size(); ++i)
          {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            pts_y[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }

          //Initialize a spline
          tk::spline s;
          s.set_points(pts_x, pts_y);
          
          //(x,y) points for the planner
          vector<double> next_x_vals,next_y_vals;

          //Add the values up to latest path 
          for (int i=0; i<previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Segmenting the spline to maintain reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
          double x_add_on = 0;

          //Setting 50 points for the path
          for (int i=1; i<=50-previous_path_x.size(); ++i)
          {
            ref_vel += speed_diff;
            if (ref_vel > 49.9)
            {
              ref_vel = 49.9;
            }
            else if (ref_vel < 0.224)
            {
              ref_vel = 0.224;
            }
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back ti normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          // ||||||||||||||||||||||||||||||||||| //
          json msgJson;

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
// |||END OF THE CODE||| //