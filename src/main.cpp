#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "json.hpp"
#include "trajectoryGenerator.hpp"
#include "trackmap.hpp"
#include "tools.hpp"
#include "vehicle.hpp"
#include "spline.h"


using namespace std;

// for convenience
using json = nlohmann::json;


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
    
  TrajectoryGenerator PTG;

  // Read waypoint map from csv file
  trackmap map("../data/highway_map.csv");

  // create object for ego vehicle;
  Vehicle ego_veh;
  
  // #################################
  // CONFIG
  // #################################
  int horizon_global = 160;
  int horizon = horizon_global;
  int update_interval_global = 20; // update every second
  int update_interval = update_interval_global;
  double speed_limit = 47.0;
  

  h.onMessage([&map, &PTG, &ego_veh, &horizon, &horizon_global, &update_interval_global, &update_interval, &speed_limit]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
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
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];

          int prev_path_size = previous_path_x.size();          
          vector<vector<double>> prevPath = {previous_path_x, previous_path_y};

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;
          
          vector<double> next_x_vals;
          vector<double> next_y_vals; 

          const int reuse_prev_range = 5;
          const double dt = 0.02; // seconds

          // wrap around cars s coordinate around max_s
          //car_s = fmod(car_s, map.max_s);

          // update actual position
          ego_veh.set_frenet_pos(car_s, car_d);

          // plan a new path
          if (prev_path_size < (horizon - update_interval))
          {
            // fit position spline for track around 10 previous and 20 next waypoints
            map.fit_spline_segment(car_s);

            // convert current s of ego and other vehicles into local frenet space
            double car_local_s = map.get_local_s(car_s);

            // turn each vehicle in the sensor fusion data into Vehicle objects
            // TODO: don't use vector, use map instead, with car ID as key!
            vector<Vehicle> other_vehicles;
            for (auto car: sensor_fusion) {
              // car idx: 0,  1, 2, 3,  4,  5, 6
              //          id, x, y, vx, vy, s, d
              Vehicle vehicle((int) car[0]);
              double local_s = map.get_local_s(car[5]);
              vehicle.set_frenet_pos(local_s, car[6]);
              double velocity_per_timestep = norm(car[3], car[4]) * dt;
              vehicle.set_frenet_motion(velocity_per_timestep, 0.0, 0.0, 0.0);
              other_vehicles.push_back(vehicle);
            }

            int lag = horizon - update_interval - prev_path_size;
            //cout << "prev_path_size: " << prev_path_size << endl;
            cout << "lag: " << lag << endl;
            if (lag > 10)
              lag = 0; // sim start

            vector<double> car_state = {car_local_s, car_d, mph2mps(car_speed)};
            cout << "car state s: " << car_s << ", d: " << car_d << ", speed (mph): " << car_speed << endl;
            auto trajectory = PTG.generate_trajectory(car_state, speed_limit, horizon, update_interval, lag, other_vehicles);
            update_interval = update_interval_global;
            horizon = horizon_global;
            if (PTG.get_current_action() == "lane_change") {
              cout << "Changing lane!" << endl;
              update_interval = horizon - 50;
            } else if (PTG.get_current_action() == "emergency") {
              cout << "Emergency!" << endl;
              horizon = 120;
              update_interval = horizon - 80;
            }
            
            // get only path in S and D frenet coordinates
            auto newPath = trajectory.getSD();
            map.getSmoothPath(prevPath, newPath, next_x_vals, next_y_vals, reuse_prev_range);

          } else {
            // only copy remaining path back to new path
            for(int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          std::string msg = "42[\"control\","+ msgJson.dump()+"]";
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
