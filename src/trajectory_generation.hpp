#ifndef TRAJECTORY_GENERATION_HPP
#define TRAJECTORY_GENERATION_HPP

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "trackmap.hpp"
#include "tools.hpp"

using namespace std;

class Vehicle {
public:

	/**
	* Constructor
	*/
	Vehicle();
	Vehicle(int lane, double s, double v, double a, string state="KL");

	/**
	* Destructor
	*/
	virtual ~Vehicle();

	vector<Vehicle> 	choose_next_state(map<int, vector<Vehicle>> predictions);
	vector<string> 		successor_states();
	vector<Vehicle> 	generate_trajectory(string state, map<int, vector<Vehicle>> predictions);
	vector<double>  	get_kinematics(map<int, vector<Vehicle>> predictions, int lane, double time_horizon = 0.0);
	vector<Vehicle> 	constant_speed_trajectory();
	vector<Vehicle> 	keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	generate_predictions(double horizon=2.0);

	void 	setPriorities(double reachGoal, double efficiency, double velocity);
	double 	position_at(double t);
	bool 	get_vehicle_behind(map<int, vector<Vehicle>> predictions,int lane, Vehicle & rVehicle);
	bool 	get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
	void 	realize_next_state(vector<Vehicle> trajectory);
	void    updateState(int lane, double s, double v, double a);
	void 	configure(double target_speed, int lanes_available, double goal_s, int goal_lane, double max_acceleration);


//private:

	map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
	int L = 1;
	int preferred_buffer = 10; // impacts "keep lane" behavior.
	int lane;
	double s;
	double v;
	double yaw;
	double a;
	double target_speed;
	int lanes_available;
	double max_acceleration;

	int goal_lane;
	double goal_s;
	string state;

	double REACH_GOAL_COST;
	double EFFICIENCY_COST;
	double VELOCITY_COST;
};


double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions,
					  const vector<Vehicle> & trajectory, float REACH_GOAL=1.0, float EFFICIENCY=1.0, float VELOCITY=1.0);
double goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,
	                      const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory,
	                     const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double velocity_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

bool findClosestCar(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, Vehicle &rVehicle);
double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);
map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory,
	                                const map<int, vector<Vehicle>> & predictions);
void getWaypointsFromTrajectory(const vector<Vehicle> &trajectory, const double timehorizon, trackmap &map_interp,
                                path_t &path, const int reuse_wp_cnt);



#endif // TRAJECTORY_GENERATION_HPP
