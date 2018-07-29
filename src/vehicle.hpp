#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <vector>
#include <iostream>
#include <cmath>

using namespace std;

class Vehicle {

public:
    Vehicle(const int id = 0);
    Vehicle(const Vehicle& src);
    ~Vehicle() {}

    void set_frenet_pos(double pos_s, double pos_d);
    void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);
    void set_frenet_state(double pos_s, double vel_s, double acc_s,
                          double pos_d, double vel_d, double acc_d);
    vector<double> get_s() const;
    vector<double> get_d() const;
    vector<double> state_at(double t) const;
    
    void configure(const int target_speed_mps, const int max_acceleration);
    
    void set_id(const int id);
    int get_id() const;

    int _target_speed_mps;
    int _max_acceleration;

private:
    int _id;
    double _pos_s;
    double _pos_d;
    double _vel_s;
    double _vel_d;
    double _acc_s;
    double _acc_d;
};

#endif // VEHICLE_HPP

