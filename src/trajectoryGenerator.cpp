#include "trajectoryGenerator.hpp"


using namespace Eigen;
using namespace std;



void Trajectory::removeFirstPoints(int numPoints) {
    if (_traj.size() < numPoints)
        return;
    
    _traj.erase(_traj.begin(), _traj.begin() + numPoints);
}

void Trajectory::removeLastPoints(int numPoints) {
    if (_traj.size() < numPoints)
        return;

    _traj.erase(_traj.end() - numPoints, _traj.end());
}

vector<double> Trajectory::operator[](int T) {
    return getState_at(T);
}

vector<double> Trajectory::getState_at(int T) {
    if (!_generated) {
        cerr << "Generate trajectory first!" << endl;
        return {};
    }
    if (T >= _traj.size()) {
        cerr << "Query at T=" << T << " out of bounds!" << endl;
        return {};
    }

    vector<double> state = _traj[T].get_s();
    auto d_vals = _traj[T].get_d();
    state.insert(state.end(), d_vals.begin(), d_vals.end());
    return state;
}


vector<vector<double>> Trajectory::getSD() {
    int traj_size = _traj.size();
    vector<double> s_list(traj_size);
    vector<double> d_list(traj_size);

    for (int i = 0; i < traj_size; i++) {
        vector<double> s_state = _traj[i].get_s();
        vector<double> d_state = _traj[i].get_d();    
        s_list[i] = s_state[0]; // only take s component
        d_list[i] = d_state[0]; // only take d component
    }        
    
    return {s_list, d_list};
}


void Trajectory::generate(pair<Polynomial, Polynomial> &traj_coeffs,
                          int samples)
{
    _traj.resize(samples);
    for (int t = 0; t < samples; t++)
    {
        double s        = traj_coeffs.first.polyeval(t);
        double s_d      = traj_coeffs.first.polyeval_d(t);
        double s_dd     = traj_coeffs.first.polyeval_dd(t);            
        double d        = traj_coeffs.second.polyeval(t);
        double d_d      = traj_coeffs.second.polyeval_d(t);
        double d_dd     = traj_coeffs.second.polyeval_dd(t);
        _traj[t].set_frenet_state(s, s_d, s_dd, d, d_d, d_dd);
    }
    _generated = true;
}




/////////////////////////////////////////////////////////////////
// Cost functions START
/////////////////////////////////////////////////////////////////
double TrajectoryGenerator::exceeds_speed_limit_cost(const pair<Polynomial, Polynomial> &traj,
                                                     const vector<double> &goal,
                                                     const vector<Vehicle> &vehicles)
{
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.polyeval_d(i) + traj.second.polyeval_d(i) > _hard_max_vel_per_timestep) {
      //cout << "exceeds_speed_limit_cost!" << endl;
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::exceeds_accel_cost(const pair<Polynomial, Polynomial> &traj,
                                               const vector<double> &goal,
                                               const vector<Vehicle> &vehicles)
{
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.polyeval_dd(i) + traj.second.polyeval_dd(i) > _hard_max_acc_per_timestep) {
      //cout << "exceeds_accel_cost!" << endl;
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::exceeds_jerk_cost(const pair<Polynomial, Polynomial> &traj,
                                              const vector<double> &goal,
                                              const vector<Vehicle> &vehicles)
{
  for (int i = 0; i < _horizon; i++) {
    if (traj.first.polyeval_ddd(i) + traj.second.polyeval_ddd(i) > _hard_max_jerk_per_timestep) {
      //cout << "exceeds_accel_cost!" << endl;
      return 1.0;
    }
  }
  return 0.0;
}

double TrajectoryGenerator::collision_cost(const pair<Polynomial, Polynomial> &traj,
                                           const vector<double> &goal,
                                           const vector<Vehicle> &vehicles)
{
  for (int t = 0; t < _horizon; t++)
  {
    for (int i = 0; i < vehicles.size(); i++)
    {
      double ego_s = traj.first.polyeval(t);
      double ego_d = traj.second.polyeval(t);
      vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}
      double diff_s = abs(traffic_state[0] - ego_s);
      double diff_d = abs(traffic_state[1] - ego_d);
      
      // make the envelope a little wider to stay "out of trouble"
      if ((diff_s <= _col_buf_length) && (diff_d <= _col_buf_width)) {
        //cout << "trajectory contains collision! (s: " << ego_s << ", d: " << ego_d << ")" << endl;
        return 1.0;
      }
    }
  }
  return 0.0;
}

// adds cost for getting too close to another vehicle
double TrajectoryGenerator::traffic_buffer_cost(const pair<Polynomial, Polynomial> &traj,
                                                const vector<double> &goal,
                                                const vector<Vehicle> &vehicles)
{
  double cost = 0.0;
  for (int i = 0; i < vehicles.size(); i++) {
    for (int t = 0; t < _horizon; t++) {
      double ego_s = traj.first.polyeval(t);
      double ego_d = traj.second.polyeval(t);
      vector<double> traffic_state = vehicles[i].state_at(t); 
      double diff_s = traffic_state[0] - ego_s;
      // ignore (potentially faster) vehicles from behind
      if (diff_s < -10) {
        break;
      }
      diff_s = abs(diff_s);
      double diff_d = abs(traffic_state[1] - ego_d);
      
      // if in the same lane and too close
      if ((diff_s <= _col_buf_length) && (diff_d <= _col_buf_width)) {
        cost += logistic(1 - (diff_s / _col_buf_length)) / _horizon;
      }
    }
  }
  return cost;
}

// penalizes low average speeds compared to speed limit
double TrajectoryGenerator::efficiency_cost(const pair<Polynomial, Polynomial> &traj,
                                            const vector<double> &goal,
                                            const vector<Vehicle> &vehicles)
{
  double s_dist = goal[0] - traj.first.polyeval(0);
  double max_dist = _delta_s_maxspeed;
  return abs(logistic((max_dist - s_dist) / max_dist)); // abs() because going faster is actually bad
}

double TrajectoryGenerator::total_accel_s_cost(const pair<Polynomial, Polynomial> &traj,
                                               const vector<double> &goal,
                                               const vector<Vehicle> &vehicles)
{
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.polyeval_dd(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::total_accel_d_cost(const pair<Polynomial, Polynomial> &traj,
                                               const vector<double> &goal,
                                               const vector<Vehicle> &vehicles)
{
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.second.polyeval_dd(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::total_jerk_cost(const pair<Polynomial, Polynomial> &traj,
                                            const vector<double> &goal,
                                            const vector<Vehicle> &vehicles)
{
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    cost += abs(traj.first.polyeval_ddd(t));
    cost += abs(traj.second.polyeval_ddd(t));
  }
  return logistic(cost);
}

double TrajectoryGenerator::lane_depart_cost(const pair<Polynomial, Polynomial> &traj,
                                             const vector<double> &goal,
                                             const vector<Vehicle> &vehicles)
{
  double cost = 0.0;
  for (int t = 0; t < _horizon; t++) {
    double ego_d = traj.second.polyeval(t);
    double lane_marking_proximity = fmod(ego_d, 4);

    if (lane_marking_proximity > 2.0) {
      lane_marking_proximity = abs(lane_marking_proximity - 4);
    }
    // car touches middle lane
    if (lane_marking_proximity <= _car_col_width)  {
      cost += 1 - logistic(lane_marking_proximity);
    }
  }
  return cost;
}

// nudges vehicle to proactively depart lanes with traffic ahead and prevent changing into busy lanes
double TrajectoryGenerator::traffic_ahead_cost(const pair<Polynomial, Polynomial> &traj,
                                               const vector<double> &goal,
                                               const vector<Vehicle> &vehicles)
{
  double ego_s = traj.first.polyeval(0);
  double ego_d = traj.second.polyeval(0);
  double ego_d_end = traj.second.polyeval(_horizon);
  int look_ahead_s = 150;

  int future_lane = convert_d_to_lane(ego_d_end);
  int closest_veh_future_lane = closest_vehicle_in_lane({ego_s, ego_d_end}, future_lane, vehicles);
  // if there is a vehicle in the lane the trajectory will take us to
  if (closest_veh_future_lane != -1) {
    vector<double> fut_traffic_s = vehicles[closest_veh_future_lane].get_s();
    float diff_s = fut_traffic_s[0] - ego_s;
    if (diff_s < look_ahead_s) {
      // don't switch into a lane with slower traffic ahead
      int cur_lane = convert_d_to_lane(ego_d);
      int closest_vehicle = closest_vehicle_in_lane({ego_s, ego_d_end}, cur_lane, vehicles);
      // if there is a vehicle in the current lane AND make range a bit tighter
      if ((closest_vehicle != -1) && (diff_s < look_ahead_s / 2.0)) {
        vector<double> traffic_s = vehicles[closest_vehicle].get_s();
        double ego_s_vel = traj.first.polyeval_d(0);
        // traffic in planned lane clearly slower than in current?
        if (fut_traffic_s[1] < traffic_s[1] * 0.9) {
          return 1e3;
        }
      }
      // end - don't switch into a lane with slower traffic ahead
      
      // no slower traffic in goal lane, so return regular cost
      return logistic(1 - (diff_s / look_ahead_s));
    }
  }
  return 0.0;
}

/////////////////////////////////////////////////////////////////
// Cost funtions END
/////////////////////////////////////////////////////////////////



double TrajectoryGenerator::calculate_cost(const pair<Polynomial, Polynomial> &traj,
                                           const vector<double> &goal,
                                           const vector<Vehicle> &vehicles,
                                           vector<vector<double>> &all_costs)
{
  Polynomial s = traj.first;
  Polynomial d = traj.second;

  // first situations that immediately make a trajectory infeasible
  double infeasible_costs =   exceeds_speed_limit_cost(traj, goal, vehicles) + exceeds_accel_cost(traj, goal, vehicles)
                            + exceeds_jerk_cost(traj, goal, vehicles) + collision_cost(traj, goal, vehicles);
  if (infeasible_costs > 0.0) {
    all_costs.push_back({1e10});
    return 1e10;
  }  

  double tr_buf_cost   = traffic_buffer_cost(traj, goal, vehicles) * _cost_weights["tr_buf_cost"];
  double eff_cost      = efficiency_cost(traj, goal, vehicles) * _cost_weights["eff_cost"];
  double acc_s_cost    = total_accel_s_cost(traj, goal, vehicles) * _cost_weights["acc_s_cost"];
  double acc_d_cost    = total_accel_d_cost(traj, goal, vehicles) * _cost_weights["acc_d_cost"];
  double jerk_cost     = total_jerk_cost(traj, goal, vehicles) * _cost_weights["jerk_cost"];
  double lane_dep_cost = lane_depart_cost(traj, goal, vehicles) * _cost_weights["lane_dep_cost"];
  double traffic_cost  = traffic_ahead_cost(traj, goal, vehicles) * _cost_weights["traffic_cost"];
  
  vector<double> cost_vec = {tr_buf_cost, eff_cost, acc_s_cost, acc_d_cost, jerk_cost, lane_dep_cost, traffic_cost};
  all_costs.push_back(cost_vec);
  
  double cost = tr_buf_cost + eff_cost + acc_s_cost + acc_d_cost + jerk_cost + lane_dep_cost + traffic_cost;
  return cost;
}


// returns a value between 0 and 1 for x in the range [0, infinity]
// and -1 to 1 for x in the range [-infinity, infinity].
// approaches 1 at an input of around 5
double TrajectoryGenerator::logistic(double x) {
  return (2.0 / (1 + exp(-x)) - 1.0);
}


int TrajectoryGenerator::convert_d_to_lane(const double d, const double laneWidth) {
  for (int lane = 0; lane <= 2; lane++) {
    if (d > laneWidth*lane && d <= laneWidth*(lane+1)) {
      return lane;
    }
  }
  return 0;
}


double TrajectoryGenerator::convert_lane_to_d(const int lane_number, const double laneWidth) {
  return (laneWidth * lane_number) + 2.0;
}


// searches for closest vehicle in current travel lane. Returns index and s-distance.
int TrajectoryGenerator::closest_vehicle_in_lane(const vector<double> &start, const int ego_lane,
                                                 const vector<Vehicle> &vehicles)
{
  int closest_vehicle = -1;
  float min_s_dif = 1e10;
  for (int i = 0; i < vehicles.size(); i++) {
    vector<double> traffic_d = vehicles[i].get_d();
    int traffic_lane = convert_d_to_lane(traffic_d[0]);
    if (ego_lane == traffic_lane) {
      vector<double> traffic_s = vehicles[i].get_s();
      float diff_s = traffic_s[0] - start[0];
      if ((diff_s > 0.0) && (diff_s < min_s_dif)) {
        closest_vehicle = i;
        min_s_dif = diff_s;
      }        
    }
  }
  return closest_vehicle;
}


// searches for closest vehicle in current travel lane. Returns index and s-distance.
vector<int> TrajectoryGenerator::closest_vehicle_in_lanes(const vector<double> &start,
                                                          const vector<Vehicle> &vehicles)
{
  vector<int> closest_vehicles(3);
  for (int lane = 0; lane < 3; lane++) {
    closest_vehicles[lane] = closest_vehicle_in_lane(start, lane, vehicles);
  }
  return closest_vehicles;
}


string TrajectoryGenerator::get_current_action() const {
  return _current_action;
}

Trajectory TrajectoryGenerator::get_last_trajectory() const {
  return _trajectory;
}

// returns: trajectory for given number of timesteps (horizon) in frenet coordinates
Trajectory TrajectoryGenerator::generate_trajectory(const vector<double> &start,
                                                    const double max_speed,
                                                    const double horizon,
                                                    const double update_interval,
                                                    const int lag,
                                                    const vector<Vehicle> &vehicles)
{
  vector<double> start_s;
  vector<double> start_d;
  double car_s = start[0];
  double car_d = start[1];
  double car_speed = start[2];
  // if we just started
  if (lag == 0) {
    start_s = {car_s, car_speed,  0.0};
    start_d = {car_d, 0.0,        0.0};
  } else {
    auto last_sample = _trajectory[lag+update_interval];
    start_s = {car_s, last_sample[1], last_sample[2]};
    start_d = {car_d, last_sample[4], last_sample[5]};
  }

  _horizon = horizon;
  double delta_s_test = (pow(mph2mps(max_speed)*_dt, 2) - pow(start_s[1], 2)) / (2. * _hard_max_acc_per_timestep);
  //cout << "delta_s_test = " << delta_s_test << endl;
  // calculating weighted average speed between v_0 and v_max
  _max_dist_per_timestep = (1*mph2mps(max_speed)*_dt + 2*start_s[1]) / 3.;
  //cout << "_max_dist_per_timestep: "  << _max_dist_per_timestep << endl;
  _delta_s_maxspeed = _max_dist_per_timestep * horizon;
  //cout << "_delta_s_maxspeed: "  << _delta_s_maxspeed << endl;  
  int cur_lane = convert_d_to_lane(start_d[0]);
  cout << "local_s: " << start_s[0] << " s_vel: " << start_s[1] << " d: " << start_d[0] << endl;

  vector<double> goal_vec;
  vector<vector<double>> goal_points;
  vector<vector<double>> traj_goals;
  vector<double> traj_costs;
  vector<string> traj_states;


  // Find feasable next states from previous states:
  // - go straight
  // - go straight and follow vehicle in front
  // - lane change left
  // - lane change right
  bool go_straight = true;
  bool go_straight_follow_lead = false;
  bool change_left  = false;
  bool change_right = false;
  vector<int> closest_vehicle = closest_vehicle_in_lanes(start_s, vehicles);

  if (closest_vehicle[cur_lane] != -1)
  {
    cout << "closest vehicle: "  << closest_vehicle[cur_lane] 
         << ", position_s: " << vehicles[closest_vehicle[cur_lane]].get_s()[0]
         << ", position_d: " << vehicles[closest_vehicle[cur_lane]].get_d()[0] << endl;

    vector<double> closest_veh_s = vehicles[closest_vehicle[cur_lane]].get_s();
    double vehicle_dist = abs(closest_veh_s[0] - start_s[0]);
    // traffic far ahead: consider overtaking left or right already
    if (vehicle_dist < 10*_col_buf_length) {
      go_straight             = true;
      go_straight_follow_lead = false;
      change_left             = true;
      change_right            = true;
    }
    // a vehicle is close ahead: either follow or overtake left or right
    if (vehicle_dist < _col_buf_length) {
      go_straight             = false;
      go_straight_follow_lead = true;
      change_left             = true;
      change_right            = true;
    }
  }
  
  // Special case:
  //   Vehicle is in inner/outer lane with traffic, middle lane has similar traffic but opposite lane is free.
  //   Need to get to the middle lane in order to "escape".
  bool prefer_mid_lane = false;
  if ((cur_lane == 0) || (cur_lane == 2))
  {
    vector<double> closest_veh_s_curLane  = vehicles[closest_vehicle[cur_lane]].get_s();
    vector<double> closest_veh_s_midLane  = vehicles[closest_vehicle[1]].get_s();
    vector<double> closest_veh_s_opLane   = vehicles[closest_vehicle[abs(cur_lane - 2)]].get_s();
    // traffic in current lane
    if (abs(closest_veh_s_curLane[0] - start_s[0]) <  2 * _col_buf_length)
    {
      // traffic in middle lane
      if (abs(closest_veh_s_midLane[0] - start_s[0]) <  2 * _col_buf_length)
      {
        // no traffic on opposite lane
        if (abs(closest_veh_s_opLane[0] - start_s[0]) >  4 * _col_buf_length)
        {
          prefer_mid_lane = true;
          if (cur_lane == 0) {
            change_right = true;
          } else {
            change_left = true;
          }
        }
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////
  // Generate goal positions for trajectories
  /////////////////////////////////////////////////////////////////
  // Go straight
  if (go_straight) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    double goal_s_acc = 0.0;
    double goal_d_pos = convert_lane_to_d(cur_lane);
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    goal_points.push_back(goal_vec);
    perturb_goal(goal_vec, goal_points);
    for (int i = 0; i <= _goal_perturb_samples; i++)
      traj_states.push_back("straight");
  }
  
  // Follow vehicle in front
  if (go_straight_follow_lead) {
    vector<double> lead_s = vehicles[closest_vehicle[cur_lane]].get_s();
    
    // Emergency break, if much slower vehicle pulls into lane dangerously close in front of us
    if (((lead_s[0] - start_s[0]) < _col_buf_length * 0.5) && (lead_s[1] < start_s[1] * 0.8)) {
      cout << "EMERGENCY" << endl;
      // reducing horizon to reduce speed faster
      _current_action = "emergency";
      _horizon = 120;
      // and hold lane - getting forced into other lane in a small horizon will exceed force limits
      change_left = false;
      change_right = false;
    }      
    double goal_s_pos = start_s[0] + lead_s[1] * _horizon;
    double goal_s_vel = lead_s[1];
    double goal_s_acc = 0.0;
    double goal_d_pos = convert_lane_to_d(cur_lane);
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    goal_points.push_back(goal_vec);
    perturb_goal(goal_vec, goal_points);
    for (int i = 0; i <= _goal_perturb_samples; i++)
      if (_current_action == "emergency") {
        traj_states.push_back("emergency");
      } else {
        traj_states.push_back("follow_straight");
      }
  }
  
  // change lane left
  if (change_left && (cur_lane != 0)) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    // less aggressive lane change if we are already following
    if (go_straight_follow_lead) {
      vector<double> lead_s = vehicles[closest_vehicle[cur_lane]].get_s();
      // but only if following closely
      if (lead_s[0] - start_s[0] < _col_buf_length * 0.5) {
        goal_s_pos = start_s[0] + lead_s[1] * _horizon;
        goal_s_vel = lead_s[1];
      }
    }
    double goal_s_acc = 0.0;
    double goal_d_pos = convert_lane_to_d(cur_lane - 1);
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    goal_points.push_back(goal_vec);
    perturb_goal(goal_vec, goal_points);
    for (int i = 0; i <= _goal_perturb_samples; i++)
      traj_states.push_back("left");      
  }

  // change lane right
  if (change_right && (cur_lane != 2)) {
    double goal_s_pos = start_s[0] + _delta_s_maxspeed;
    double goal_s_vel = _max_dist_per_timestep;
    // less aggressive lane change if we are already following
    if (go_straight_follow_lead) {
      vector<double> lead_s = vehicles[closest_vehicle[cur_lane]].get_s();
      // but only if following closely
      if (lead_s[0] - start_s[0] < _col_buf_length * 0.5) {
        goal_s_pos = start_s[0] + lead_s[1] * _horizon;
        goal_s_vel = lead_s[1];
      }
    }
    double goal_s_acc = 0.0;
    double goal_d_pos = convert_lane_to_d(cur_lane + 1);
    double goal_d_vel = 0.0;
    double goal_d_acc = 0.0;
    goal_vec = {goal_s_pos, goal_s_vel, goal_s_acc, goal_d_pos, goal_d_vel, goal_d_acc};
    goal_points.push_back(goal_vec);
    perturb_goal(goal_vec, goal_points);
    for (int i = 0; i <= _goal_perturb_samples; i++)
      traj_states.push_back("right");      
  }

  cout << "Possible next state(s): ";
  if (go_straight)
    cout << " GO STRAIGHT ";
  if (go_straight_follow_lead)
    cout << " FOLLOW LEAD ";
  if (change_left)
    cout << " CHANGE LEFT ";
  if (change_right)
    cout << " CHANGE RIGHT ";
  cout << endl;
  
  // Generate jerk minimal trajectories
  vector<pair<Polynomial, Polynomial>> traj_coeffs;
  for (vector<double> goal : goal_points) {
    vector<double> goal_s = {goal[0], goal[1], goal[2]};
    vector<double> goal_d = {goal[3], goal[4], goal[5]};
    // d should be within bounds of the road
    if ((goal[3] > 1.0) && (goal[3] < 11.0)) {      
      Polynomial traj_s_poly = jmt(start_s, goal_s, _horizon);
      Polynomial traj_d_poly = jmt(start_d, goal_d, _horizon);
      traj_coeffs.push_back(std::make_pair(traj_s_poly, traj_d_poly));
      traj_goals.push_back({goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]});
    }
  }
  
  // Calculate cost for each trajectory
  vector<vector<double>> all_costs;
  for (int i = 0; i < traj_coeffs.size(); i++) {
    double cost = calculate_cost(traj_coeffs[i], traj_goals[i], vehicles, all_costs);
    // if applicable, reduce costs of a lane change that gets us closer to the lane with less traffic
    if (prefer_mid_lane && (abs(convert_lane_to_d(1) - traj_coeffs[i].second.polyeval(_horizon)) < 1.0)) {
      cost *= 0.5;
    }
    traj_costs.push_back(cost);
  }
    
  // select least-cost trajectory
  double min_cost = traj_costs[0];
  int min_cost_traj_num = 0;
  for (int i = 1; i < traj_coeffs.size(); i++) {
    if (traj_costs[i] < min_cost) {
      min_cost = traj_costs[i];
      min_cost_traj_num = i;
    }
  }

  // dealing with rare corner case: vehicle is stuck in infeasible trajectory (usually stuck close behind other car)
  if (min_cost == 1e10) {
    double min_s = traj_coeffs[0].first.polyeval(_horizon);
    int min_s_num = 0;
    // find generated trajectory going straight with minimum s
    for (int i = 1; i <= _goal_perturb_samples; i++) {
      if (traj_coeffs[i].first.polyeval(_horizon) < min_s){
        min_s = traj_coeffs[i].first.polyeval(_horizon);
        min_s_num = i;
      }
    }
    min_cost_traj_num = min_s_num;
  }
  
  _current_action = "straight";
  if (traj_states[min_cost_traj_num] == "left" || traj_states[min_cost_traj_num] == "right") {
    _current_action = "lane_change";  
  }

  /*
  for (int i = 0;  i < traj_states.size(); i++) {
    cout << "cost[" << traj_states[i] << "]: " << traj_costs[i] << endl;
    if (i < all_costs.size()) {
      if (all_costs[i].size() >= 6) {
        cout << "\ttraffic buffer cost: " << all_costs[i][0] << endl;
        cout << "\tefficiency cost: " << all_costs[i][1] << endl;
        cout << "\tacceleration s cost: " << all_costs[i][2] << endl;
        cout << "\tacceleration d cost: " << all_costs[i][3] << endl;
        cout << "\tjerk cost: " << all_costs[i][4] << endl;
        cout << "\tlane depart cost: " << all_costs[i][5] << endl;
        cout << "\ttraffic cost: " << all_costs[i][6] << endl;
        cout << "\tlowest cost traj goal s/d: " << goal_points[i][0]
             << " / " << goal_points[i][3] << endl;
      }
    }    
  }
  */

  cout << "chosen maneuver:     " << traj_states[min_cost_traj_num] << " (s: "
       << goal_points[min_cost_traj_num][0] << " / d: "
       << goal_points[min_cost_traj_num][3] << ")"  << endl;
  cout << "traffic buffer cost: " << all_costs[min_cost_traj_num][0] << endl;
  cout << "efficiency cost:     " << all_costs[min_cost_traj_num][1] << endl;
  cout << "acceleration s cost: " << all_costs[min_cost_traj_num][2] << endl;
  cout << "acceleration d cost: " << all_costs[min_cost_traj_num][3] << endl;
  cout << "jerk cost:           " << all_costs[min_cost_traj_num][4] << endl;
  cout << "lane depart cost:    " << all_costs[min_cost_traj_num][5] << endl;
  cout << "traffic cost:        " << all_costs[min_cost_traj_num][6] << endl;

  // eventually generate trajectory
  _trajectory.generate(traj_coeffs[min_cost_traj_num], _horizon);
  return _trajectory;
}


// creates randomly generated variations of goal point
void TrajectoryGenerator::perturb_goal(vector<double> goal,
                                       vector<vector<double>> &goal_points,
                                       bool no_ahead)
{
  double percentage_std_deviation = 0.1;
  std::normal_distribution<double> distribution_10_percent(0.0, percentage_std_deviation);
  vector<double> pert_goal(6);
  for (int i = 0; i < _goal_perturb_samples; i++) {
    double multiplier = distribution_10_percent(_rand_generator);
    if (no_ahead && (multiplier > 0.0)) {
      multiplier *= -1.0;
    }
    pert_goal[0] = goal[0] + (_delta_s_maxspeed * multiplier);
    pert_goal[1] = goal[1] + (_max_dist_per_timestep * multiplier);
    pert_goal[2] = 0.0;
    
    multiplier = distribution_10_percent(_rand_generator);
    pert_goal[3] = goal[3] + multiplier;
    pert_goal[4] = 0.0;
    pert_goal[5] = 0.0;
    goal_points.push_back(pert_goal);
  }
}


Polynomial TrajectoryGenerator::jmt(const vector<double> &start,
                                    const vector<double> &end,
                                    const int T)
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
    an instance of class Polynomial of the following form:
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

    VectorXd b = A.inverse() * c;
    Polynomial result({ start[0], start[1], 0.5*start[2], b(0), b(1), b(2) });
    return result;
}