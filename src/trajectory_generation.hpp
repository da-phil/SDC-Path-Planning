#ifndef TRAJECTORY_GENERATION_HPP
#define TRAJECTORY_GENERATION_HPP

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

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
	vector<double>  	get_kinematics(map<int, vector<Vehicle>> predictions, int lane);
	vector<Vehicle> 	constant_speed_trajectory();
	vector<Vehicle> 	keep_lane_trajectory(map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);
	vector<Vehicle> 	generate_predictions(int horizon=2);

	void 	setPriorities(double reachGoal, double efficiency);
	void 	increment(int dt);
	double 	position_at(int t);
	bool 	get_vehicle_behind(map<int, vector<Vehicle>> predictions,int lane, Vehicle & rVehicle);
	bool 	get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
	void 	realize_next_state(vector<Vehicle> trajectory);
	void    updateState(int lane, double s, double v, double a, string state="KL");
	void 	configure(int target_speed, int lanes_available, int goal_s, int goal_lane, int max_acceleration);


//private:

	map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

	struct collider{
		bool collision ; 	// is there a collision?
		int  time; 			// time collision happens
	};

	int L = 1;
	int preferred_buffer = 6; // impacts "keep lane" behavior.
	int lane;
	int s;
	double v;
	double yaw;
	double a;
	double target_speed;
	int lanes_available;
	double max_acceleration;
	int goal_lane;
	int goal_s;
	string state;

	double REACH_GOAL = 6;
	double EFFICIENCY = 1;
};


double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions,
					  const vector<Vehicle> & trajectory, float REACH_GOAL=6, float EFFICIENCY=1);
double goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,
	                      const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory,
	                     const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);
map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory,
	                                const map<int, vector<Vehicle>> & predictions);


#endif // TRAJECTORY_GENERATION_HPP
