#include "vehicle.hpp"

Vehicle::Vehicle(const int id) {
    _id     = id;
    _pos_s  = 0.0;
    _vel_s  = 0.0;
    _acc_s  = 0.0;
    _pos_d  = 0.0;
    _vel_d  = 0.0;
    _acc_d  = 0.0;
}

Vehicle::Vehicle(const Vehicle& src) {
    _pos_s = src._pos_s;
    _vel_s = src._vel_s;
    _acc_s = src._acc_s;
    _pos_d = src._pos_d;
    _vel_d = src._vel_d;
    _acc_d = src._acc_d;  
    _target_speed_mps = src._target_speed_mps;
    _max_acceleration = src._max_acceleration;
}

void Vehicle::configure(const int target_speed_mps, const int max_acceleration) {
    _target_speed_mps = target_speed_mps;
    _max_acceleration = max_acceleration;
}

void Vehicle::set_frenet_pos(double pos_s, double pos_d) {
    _pos_s = pos_s;
    _pos_d = pos_d;
}

void Vehicle::set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d) {
    _vel_s = vel_s;
    _acc_s = acc_s;
    _vel_d = vel_d;
    _acc_d = acc_d;
}

void Vehicle::set_frenet_state(double pos_s, double vel_s, double acc_s,
                               double pos_d, double vel_d, double acc_d)
{
    set_frenet_pos(pos_s, pos_d);
    set_frenet_motion(vel_s, acc_s, vel_d, acc_d);
}

vector<double> Vehicle::get_s() const {
    return {_pos_s, _vel_s, _acc_s};
}

vector<double> Vehicle::get_d() const {
    return {_pos_d, _vel_d, _acc_d};
}

void Vehicle::set_id(const int id) {
    _id = id;
}

int Vehicle::get_id() const {
    return _id;
}

// returns frenet coordinates of predicted position at time t
// assumption in line with project constraints:
// - vehicle stays in lane
// - vehicle has constant speed
vector<double> Vehicle::state_at(double t) const {
    double new_s = _pos_s + t * _vel_s;
    return {new_s, _pos_d};
}