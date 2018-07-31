#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <random>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "polynomials.hpp"
#include "tools.hpp"
#include "vehicle.hpp"

using namespace Eigen;
using namespace std;


class Trajectory {

public:

    Trajectory() : _generated(false) {}

    Trajectory(pair<Polynomial, Polynomial> &traj_coeffs,
               int samples)
    {
        generate(traj_coeffs, samples);
    }

    ~Trajectory() {}

    void clear() {
        _traj.clear();
    }

    int size(void) const {
        return _traj.size();
    }
    
    void removeFirstPoints(int numPoints);
    void removeLastPoints(int numPoints);
    vector<double> operator[](int T);
    vector<double> getState_at(int T);
    vector<vector<double>> getSD();
    void generate(pair<Polynomial, Polynomial> &traj_coeffs,
                  int samples);

private:
    vector<Vehicle> _traj;
    bool _generated = false;
};




class TrajectoryGenerator {
    
public:
    TrajectoryGenerator() {}
    ~TrajectoryGenerator() {}

    Trajectory get_last_trajectory() const;
    Trajectory generate_trajectory(const vector<double> &start, const double max_speed,
                                   const double horizon, const double update_interval, const int lag,
                                   const vector<Vehicle> &vehicles);

    static int convert_d_to_lane(const double d, const double laneWidth = 4.0);
    static double convert_lane_to_d(const int lane_number, const double laneWidth = 4.0);
    static double logistic(const double x);
    string get_current_action() const;


private:
    Polynomial jmt(const vector<double> &start, const vector<double> &end, const int T);
    void perturb_goal(vector<double> goal, vector<vector<double>> &goal_points, bool no_ahead=false);

    int closest_vehicle_in_lane(const vector<double> &start, const int ego_lane_i,
                                const vector<Vehicle> &vehicles);
    vector<int> closest_vehicle_in_lanes(const vector<double> &start, const vector<Vehicle> &vehicles);

    // cost functions:
    double calculate_cost(const pair<Polynomial, Polynomial> &traj, const vector<double> &goal,
                          const vector<Vehicle> &vehicles, vector<vector<double>> &all_costs);
    double exceeds_speed_limit_cost(const pair<Polynomial, Polynomial> &traj,
                                    const vector<double> &goal, const vector<Vehicle> &vehicles);
    double exceeds_accel_cost(const pair<Polynomial, Polynomial> &traj,
                              const vector<double> &goal, const vector<Vehicle> &vehicles);
    double exceeds_jerk_cost(const pair<Polynomial, Polynomial> &traj,
                             const vector<double> &goal, const vector<Vehicle> &vehicles);
    double collision_cost(const pair<Polynomial, Polynomial> &traj,
                          const vector<double> &goal, const vector<Vehicle> &vehicles);
    double traffic_buffer_cost(const pair<Polynomial, Polynomial> &traj,
                               const vector<double> &goal, const vector<Vehicle> &vehicles);
    double efficiency_cost(const pair<Polynomial, Polynomial> &traj,
                           const vector<double> &goal, const vector<Vehicle> &vehicles);
    double total_accel_d_cost(const pair<Polynomial, Polynomial> &traj,
                              const vector<double> &goal, const vector<Vehicle> &vehicles);
    double total_accel_s_cost(const pair<Polynomial, Polynomial> &traj,
                              const vector<double> &goal, const vector<Vehicle> &vehicles);
    double total_jerk_cost(const pair<Polynomial, Polynomial> &traj,
                           const vector<double> &goal, const vector<Vehicle> &vehicles);
    double lane_depart_cost(const pair<Polynomial, Polynomial> &traj,
                            const vector<double> &goal, const vector<Vehicle> &vehicles);
    double traffic_ahead_cost(const pair<Polynomial, Polynomial> &traj,
                              const vector<double> &goal, const vector<Vehicle> &vehicles);


    Trajectory _trajectory;
    int _horizon                    = 0;
    std::string _current_action     = "straight";
    const double _dt                = 0.02;
    const double _car_width         = 2.0;
    const double _car_length        = 5.0;
    const double _car_col_width     = 0.5 * _car_width;
    const double _car_col_length    = 0.5 * _car_length;
    const double _col_buf_width     = _car_width;
    const double _col_buf_length    = 5 * _car_length;
    const int _goal_perturb_samples = 10;
    const double _hard_max_vel_per_timestep     = mph2mps(48.) * _dt; // 50 miles + a little buffer
    const double _hard_max_acc_per_timestep     = 8.0 * _dt; // m/s
    const double _hard_max_jerk_per_timestep    = 7.0 * _dt; // m/s^2
    double _max_dist_per_timestep               = 0.0;
    double _delta_s_maxspeed                    = 0.0;
    
    std::default_random_engine _rand_generator;
    std::map<std::string, double> _cost_weights = {
        {"tr_buf_cost",     200.0},
        {"eff_cost",        100.0},
        {"acc_s_cost",      20.0},
        {"acc_d_cost",      20.0},
        {"jerk_cost",       20.0},
        {"lane_dep_cost",   0.5},
        {"traffic_cost",    100.0}
    };
};

#endif // TRAJECTORYGENERATOR_HPP


