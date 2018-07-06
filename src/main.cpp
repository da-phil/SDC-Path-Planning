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
#include "trackmap.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	  //cout << "str: " << s << endl;
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

  // Waypoint map to read from, waypoints are interpolated with cubic splines
  trackmap map_interp("../data/highway_map.csv");
	double end_path_s, end_path_d;
	double target_speed_mph = 47;
	double target_speed_mps = mph2mps(target_speed_mph);
  // Vehicle(int lane, double s, double v, double a, string state="CS")
  Vehicle ego_vehicle;
  // configure(int target_speed, int lanes_available, int goal_s, int goal_lane, int max_acceleration)
  ego_vehicle.configure(target_speed_mps, 3, max_s+1.0, 2, 6.0);
  ego_vehicle.state = "KL";

  map<int, Vehicle> other_vehicles;

  bool car_starting = true;
	path_t path;

  // setPriorities(double reachGoal, double efficiency, double velocity);
  ego_vehicle.setPriorities(6.0, 1.0, 100.); 

  h.onMessage([&other_vehicles, &ego_vehicle, &map_interp, &car_starting,
  						 &target_speed_mph, &target_speed_mps, &path]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data));
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
          vector<double> previous_path_x = j[1]["previous_path_x"].get<std::vector<double>>();
          vector<double> previous_path_y = j[1]["previous_path_y"].get<std::vector<double>>();
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          int previous_path_size = previous_path_x.size();
          int path_size = path.x.size();
          double pos_x;
          double pos_y;
          double pos_x2;
          double pos_y2;
          double angle;
          int reuse_wp_cnt = 30;
          // Wrap around cars s coordinate around max_s
          car_s = fmod(car_s, max_s);
      		
      		if (car_speed < 1e-6 && path.x.size() < 5) {
       			car_starting = true;
       		}
       		
       		//cout << "previous_path_size: " << previous_path_size << ", path_size:          " << path_size << endl;

          // add remaining waypoints from previous list into the new list
          // only start adding new waypoints if we can add more to the list
          /*
       		for (int i = 0; i < previous_path_size; i++) {
         		next_x_vals.push_back(previous_path_x[i]);
         		next_y_vals.push_back(previous_path_y[i]);
       		}
       		*/
       		// remove already used waypoints from path
       		unsigned diff = abs(path_size - previous_path_size);
       		if (diff >= 1) {
	       		path.x.erase(path.x.begin(), path.x.begin() + diff);
	       		path.y.erase(path.y.begin(), path.y.begin() + diff);
	       		path.s.erase(path.s.begin(), path.s.begin() + diff);
	       		path.d.erase(path.d.begin(), path.d.begin() + diff);
	       		path.v.erase(path.v.begin(), path.v.begin() + diff);
     			}

          cout << "----------------------------------------------------------------------------------------------------------" << endl;
          cout << "lag=" << diff << ", own_x=" << car_x << ", own_y=" << car_y << ", own_yaw=" << car_yaw << ", own_speed=" << car_speed
               << ", own_s=" << car_s << ", own_d=" << car_d << ", lane=" << getLane(car_d) << endl;

          // there is no path yet, let's gently accelerate the car until we reach the target velocity	
          if(car_starting) {
          	 path.clear();
          	 path.x.push_back(car_x);
	       		 path.y.push_back(car_y);
	       		 path.s.push_back(car_s);
	       		 path.d.push_back(car_d);
	       		 path.v.push_back(mph2mps(car_speed));
          	 pos_x = car_x;
		         pos_y = car_y;
		         pos_x2 = car_x;
		         pos_y2 = car_y;
		         angle = car_yaw;
         		 car_starting = false;
         		 cout << "starting the car!" << endl;

						 getWaypointsFromTrajectory({Vehicle(getLane(car_d), car_s, mph2mps(car_speed), 0.5),
             														 Vehicle(getLane(car_d), car_s + 1.7*target_speed_mps, target_speed_mps, 0.0)},
             														 3.5, map_interp, path, reuse_wp_cnt);
          } else if (path_size < 15) {
		          pos_x = path.x.back();
		          pos_y = path.y.back();
		          pos_x2 = path.x[path.x.size()-2];
		          pos_y2 = path.y[path.x.size()-2];
		          angle = atan2(pos_y-pos_y2,pos_x-pos_x2);


		          // update information of other cars
		        	map<int ,vector<Vehicle> > other_vehicles_predictions;
		          for (auto car: sensor_fusion) {
			            // idx: 0,  1, 2, 3,  4,  5, 6
			            //      id, x, y, vx, vy, s, d
			            int lane = getLane(car[6]);
			            /*
			            cout << "car_nr: " << car[0] << ", x=" << car[1] << ", y=" << car[2] << ", v=" << mps2mph(norm(car[3], car[4]))
			                 << ", s=" << car[5] << ", d=" << car[6] << ", lane=" << lane << endl; 
			            */
			            if (other_vehicles.find(car[0]) != other_vehicles.end()) {
			              	other_vehicles[car[0]].updateState(lane, car[5], norm(car[3], car[4]), 0.);
			            } else {
			              	other_vehicles[car[0]] = Vehicle(lane, car[5], norm(car[3], car[4]), 0., "CS");
			            }
			            vector<Vehicle> preds = other_vehicles[car[0]].generate_predictions(3.0);
			        		other_vehicles_predictions[car[0]] = preds;
		          }

	          	// update state of own car
	          	int reuse_wp_cnt_tmp = path.x.size() - 1;
	          	cout << "upadating car with d=" << path.d[reuse_wp_cnt_tmp] << ", s=" << path.s[reuse_wp_cnt_tmp] << ", v=" << mps2mph(path.v[reuse_wp_cnt_tmp]) << endl;
       	  		ego_vehicle.updateState(getLane(path.d[reuse_wp_cnt_tmp]), path.s[reuse_wp_cnt_tmp], path.v[reuse_wp_cnt_tmp], 0.);
		      	  auto trajectory = ego_vehicle.choose_next_state(other_vehicles_predictions);
		      	  ego_vehicle.realize_next_state(trajectory);
		    	  	cout << "current trajectory: " << endl;
		    	  	for (auto v: trajectory) {
		    	  		cout << "  state=" << v.state << ", lane=" << v.lane << ", s=" << v.s << ", v=" << mps2mph(v.v) << ", a=" << v.a << endl;
		    	  	}
              getWaypointsFromTrajectory(trajectory, 1.5, map_interp, path, reuse_wp_cnt);
           }


          /*
          cout << "end_path_d: " << end_path_d << ", end_path_s: " << end_path_s << endl;
					auto last = map_interp.getFrenet(pos_x, pos_y, angle);
          cout << "last_d:     " << last[1] << ", last_s: " << last[0] << endl;
 					*/
          json msgJson;
          msgJson["next_x"] = path.x;
          msgJson["next_y"] = path.y;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //cout << "sending string: " << msg << endl;
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
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
