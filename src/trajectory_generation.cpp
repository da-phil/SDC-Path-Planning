#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

#include "trajectory_generation.hpp"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){
    updateState(0, 0., 0., 0.);
}

Vehicle::Vehicle(int lane, double s, double v, double a, string state) {
    updateState(lane, s, v, a, state);
}

Vehicle::~Vehicle() {}


void Vehicle::updateState(int lane, double s, double v, double a, string state) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
    REACH_GOAL = 6;
    EFFICIENCY = 1;
}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    INPUT: A predictions map. This is a map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a vector of possible successor states for the finite 
       state machine.
    2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector of Vehicle objects 
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors 
       might have size 0 if no possible trajectory exists for the state. 
    3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory) - Included from 
       cost.cpp, computes the cost for a trajectory.
    */

    auto possible_future_states = successor_states();
    double lowest_cost = 9999;
    int cost = 0;
    vector<Vehicle> lowest_cost_trajectory = generate_trajectory(state, predictions);

    // iterate over all future states
    for (auto &future_state : possible_future_states) {
        // get trajectory into future state
        auto trajectory = generate_trajectory(future_state, predictions);
        if (trajectory.size() > 0) {
            // get summed up cost for future state
            cost = calculate_cost(*this, predictions, trajectory, REACH_GOAL, EFFICIENCY);
            if (cost < lowest_cost) {
                lowest_cost = cost;
                lowest_cost_trajectory = trajectory;
            }
        }
    }

    return lowest_cost_trajectory;
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};
}


vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    double next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
    this->s = position_at(dt);
}

double Vehicle::position_at(int dt) {
    return this->s + this->v*dt + this->a*dt*dt/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (auto &it : predictions) {
        temp_vehicle = it.second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    vector<Vehicle> predictions(horizon);
    for(int i = 0; i < horizon; i++) {
      double next_s = position_at(i);
      double next_v = 0;
      if (i < horizon-1) {
        next_v = position_at(i+1) - s;
      }
      predictions[i] = Vehicle(this->lane, next_s, next_v, 0);
    }
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(int target_speed, int lanes_available, int goal_s,
                        int goal_lane, int max_acceleration) {
    /*
    Called before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    this->target_speed = target_speed;
    this->lanes_available = lanes_available;
    this->goal_s = goal_s;
    this->goal_lane = goal_lane;
    this->max_acceleration = max_acceleration;
}


void Vehicle::setPriorities(double reachGoal, double efficiency) {
    this->REACH_GOAL = reachGoal;
    this->EFFICIENCY = efficiency;
}





/*
Here we have provided two possible suggestions for cost functions, but feel free to use your own!
The weighted cost over all cost functions is computed in calculate_cost. The data from get_helper_data
will be very useful in your implementation of the cost functions below. Please see get_helper_data
for details on how the helper data is computed. 
*/
double goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance. This function is
    very similar to what you have already implemented in the "Implement a Cost Function in C++" quiz.
    */
    double cost;
    double distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}


double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    You can use the lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) function to determine the speed
    for a lane. This function is very similar to what you have already implemented in the "Implement a Second Cost Function in C++" quiz.
    */
    double proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    //If no vehicle is in the proposed lane, we can travel at target speed.
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    double proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }
    
    double cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    return cost;
}


double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane && key != -1) {
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}


double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory,
                      float REACH_GOAL, float EFFICIENCY)
{
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
    vector<double> weight_list = {REACH_GOAL, EFFICIENCY};
    
    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;

}


map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: +/- 1 from the current lane if the ehicle is planning or executing a lane change.
    final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
    distance_to_goal: The s distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, double> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    double intended_lane;

    if (trajectory_last.state == "PLCL") {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state == "PLCR") {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    double distance_to_goal = vehicle.goal_s - trajectory_last.s;
    double final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}


vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
            corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
            length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    */

    double T2, T3, T4, T5;
    T2 = T*T;
    T3 = T2*T;
    T4 = T3*T;
    T5 = T4*T;

    MatrixXd c(3,1);
    c << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
         end[1] - (start[1] + start[2]*T),
         end[2] -  start[2];

    MatrixXd A(3,3);
    A <<   T3,    T4,    T5,
         3*T2,  4*T3,  5*T4,
         6*T,  12*T2, 20*T3;

    MatrixXd b = A.inverse() * c;
    
    return {start[0], start[1], 0.5*start[2],
            b(0),     b(1),     b(2)};
}

