#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "json.hpp"
#include "tools.hpp"
#include "trajectory_generation.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  const string map_file_ = "../data/highway_map.csv";

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
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

  // interpolate waypoint coordinates with cubic splines
  tk::spline wp_x_interp, wp_y_interp;
  wp_x_interp.set_points(map_waypoints_s, map_waypoints_x);
  wp_y_interp.set_points(map_waypoints_s, map_waypoints_y);

  // Vehicle(int lane, double s, double v, double a, string state="CS")
  Vehicle ego_vehicle;
  map<int, Vehicle> other_vehicles;

  // setPriorities(double reachGoal, double efficiency);
  ego_vehicle.setPriorities(6, 1); 


  h.onMessage([&wp_x_interp, &wp_y_interp, &other_vehicles, &ego_vehicle,
               &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
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
          double car_speed_x = car_speed * cos(deg2rad(car_yaw));
          double car_speed_y = car_speed * sin(deg2rad(car_yaw));

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Wrap around cars s coordinate around max_s
          car_s = fmod(car_s, max_s);

          // update information of other cars
          for (auto car: sensor_fusion) {
            // idx: 0,  1, 2, 3,  4,  5, 6
            //      id, x, y, vx, vy, s, d
            int lane = getLane(car[6]);
            /*
            cout << "car_nr: " << car[0] << ", x=" << car[1] << ", y=" << car[2] 
                 << ", s=" << car[5] << ", d=" << car[6] << ", lane=" << lane << endl; 
            */
            if (other_vehicles.find(car[0]) != other_vehicles.end()) {
              other_vehicles[car[0]].updateState(lane, car[5], sqrt((double)car[3]*(double)car[3]+(double)car[4]*(double)car[4]), 0., "KL");
            } else {
              other_vehicles.insert(make_pair(car[0], Vehicle(lane, car[5], sqrt((double)car[3]*(double)car[3]+(double)car[4]*(double)car[4]), 0., "KL")));
            }
            
          }
          cout << "--------------------------------------------------------------------------------" << endl;
          cout << "own_x=" << car_x << ", own_y=" << car_y << ", own_yaw=" << car_yaw << ", own_speed=" << car_speed
               << ", own_s=" << car_s << ", own_d=" << car_d << ", lane=" << getLane(car_d) << endl;

          const double time_horizon = 2.0; // seconds
          const double dt = 0.02; // seconds          
          const double target_speed_mps = 50; // miles per hour
          const double target_speed_ms = target_speed_mps * 1.6 / 3.6; // meters per second
          const double dist_inc = target_speed_ms * dt;
          const int waypoint_cnt = time_horizon / dt;

          double pos_x, pos_y, pos_s, angle;
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          int previous_path_size = previous_path_x.size();

          if(previous_path_size == 0)
          {
              pos_x = car_x;
              pos_y = car_y;
              pos_s = car_s;
              angle = deg2rad(car_yaw);
          }
          else
          {
              pos_x = previous_path_x[previous_path_size-1];
              pos_y = previous_path_y[previous_path_size-1];
              double pos_x2 = previous_path_x[previous_path_size-2];
              double pos_y2 = previous_path_y[previous_path_size-2];
              angle = atan2(pos_y-pos_y2, pos_x-pos_x2);
              auto endpoint = getFrenet(pos_x, pos_y, angle,
                                        map_waypoints_x, map_waypoints_y);
              pos_s = endpoint[0];

              // add remaining waypoints from previous list into the new list
              for(int i = 0; i < previous_path_size; i++)  {
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
              }
          }  
          // add new waypoints to new list
          for(int i = 0; i < waypoint_cnt - previous_path_size; i++) {
              next_x_vals.push_back(wp_x_interp(pos_s + i*dist_inc));
              next_y_vals.push_back(wp_y_interp(pos_s + i*dist_inc));
          }

          int cur_wp_idx = NextWaypoint(car_x, car_y, angle, map_waypoints_x, map_waypoints_y);
          cout << "previous_path_size: " << previous_path_size << endl;
          cout << "next wp idx: " << cur_wp_idx << ", x=" << map_waypoints_x[cur_wp_idx]
               << ", y=" << map_waypoints_y[cur_wp_idx] << endl;


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //cout << "sending string: " << msg << endl;
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
