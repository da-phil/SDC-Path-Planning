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


void trajectory_lanechange(const vector<double> start_s,  const vector<double> start_d,
                           const vector<double> end_s,    const double T,  const int lane_number,
                           trackmap &map_interp,          vector<double> &next_x_vals,  vector<double> &next_y_vals)
{
    double dt = 0.02;
    auto s_params = JMT(start_s, end_s, T);
    auto d_params = JMT(start_d, {getLaneOffsetD(lane_number), 0.0, 0.0}, T);              
    int waypoint_cnt = T / dt;

    //cout << "sending " << waypoint_cnt << " waypoints!" << endl;

    vector<double> s_list(waypoint_cnt+1);
    vector<double> d_list(waypoint_cnt+1);  
    for(int i = 0; i <= waypoint_cnt; i++) { 
        s_list[i] = fmod(polyeval(s_params, dt*i), max_s);
        d_list[i] = polyeval(d_params, dt*i);
        //cout << "new_s: " << s_list[i] << ", new_d: " << d_list[i] << endl;
    }
    map_interp.getXYMapAtSD(s_list, d_list, next_x_vals, next_y_vals);
}


int main() {
  uWS::Hub h;

  // Waypoint map to read from, waypoints are interpolated with cubic splines
  trackmap map_interp("../data/highway_map.csv");
	double end_path_s, end_path_d;
	double target_speed_mph = 46;
	double target_speed_mps = mph2mps(target_speed_mph);
  // Vehicle(int lane, double s, double v, double a, string state="CS")
  Vehicle ego_vehicle;
  // configure(int target_speed, int lanes_available, int goal_s, int goal_lane, int max_acceleration)
  ego_vehicle.configure(target_speed_mps, 3, max_s+1.0, 0, 4.0);
  ego_vehicle.state = "KL";

  map<int, Vehicle> other_vehicles;

  bool car_starting = true;

  // setPriorities(double reachGoal, double efficiency);
  ego_vehicle.setPriorities(6, 1); 

  h.onMessage([&other_vehicles, &ego_vehicle, &map_interp, &end_path_s, &end_path_d, &car_starting,
  						 &target_speed_mph, &target_speed_mps]
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
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Wrap around cars s coordinate around max_s
          car_s = fmod(car_s, max_s);

          cout << "--------------------------------------------------------------------------------" << endl;
          cout << "own_x=" << car_x << ", own_y=" << car_y << ", own_yaw=" << car_yaw << ", own_speed=" << car_speed
               << ", own_s=" << car_s << ", own_d=" << car_d << ", lane=" << getLane(car_d) << endl;


          vector<double> next_x_vals;
          vector<double> next_y_vals;
          int previous_path_size = previous_path_x.size();

          // add remaining waypoints from previous list into the new list
          // only start adding new waypoints if we can add more to the list
       		for (int i = 0; i < previous_path_size; i++) {
         		next_x_vals.push_back(previous_path_x[i]);
         		next_y_vals.push_back(previous_path_y[i]);
       		}

          // there is no path yet, let's gently accelerate the car until we reach the target velocity	
          if(car_starting) {
         		 car_starting = false;
             trajectory_lanechange({car_s, mph2mps(car_speed), 0.1},   {car_d, 0.1, 0.1},
                                   {car_s + 40.0, target_speed_mps, 0.0},  3.5,
                                   getLane(car_d), map_interp, next_x_vals, next_y_vals);

          } else if (previous_path_size < 10) {
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
			              	other_vehicles[car[0]].updateState(lane, car[5], norm(car[3], car[4]), 0., "KL");
			            } else {
			              	other_vehicles[car[0]] = Vehicle(lane, car[5], norm(car[3], car[4]), 0., "KL");
			            }
			            vector<Vehicle> preds = other_vehicles[car[0]].generate_predictions();
			        		other_vehicles_predictions[car[0]] = preds;
		          }

	          	// update state of own car
       	  		ego_vehicle.updateState(getLane(end_path_d), end_path_s, mph2mps(car_speed), 0.);
		      	  auto trajectory = ego_vehicle.choose_next_state(other_vehicles_predictions);
		      	  ego_vehicle.realize_next_state(trajectory);
		    	  	cout << "current trajectory: " << endl;
		    	  	for (auto v: trajectory) {
		    	  		cout << "  state=" << v.state << ", lane=" << v.lane << ", s=" << v.s << ", v=" << v.v << ", a=" << v.a << endl;
		    	  	}

							trajectory_lanechange({trajectory[0].s,  trajectory[0].v, trajectory[0].a},
																		{getLaneOffsetD(trajectory[0].lane), 0.0,  0.0},
								                    {trajectory[1].s,   trajectory[1].v,  trajectory[1].a}, 1.,
							  	                  trajectory[0].lane, map_interp, next_x_vals, next_y_vals);
           }


          /*
          cout << "end_path_d: " << end_path_d << ", end_path_s: " << end_path_s << endl;
					auto last = map_interp.getFrenet(pos_x, pos_y, angle);
          cout << "last_d:     " << last[1] << ", last_s: " << last[0] << endl;
          int cur_wp_idx = map_interp.NextWaypoint(car_x, car_y, angle);
          cout << "previous_path_size: " << previous_path_size << endl;
          cout << "next wp idx: " << cur_wp_idx << ", x=" << map_interp.getWpX(cur_wp_idx)
               << ", y=" << map_interp.getWpY(cur_wp_idx) << endl;
					*/
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
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
