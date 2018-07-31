#include "trackmap.hpp"

using namespace std;


trackmap::trackmap(const vector<double> &map_x_wp,  const vector<double> &map_y_wp,
                   const vector<double> &map_dx_wp, const vector<double> &map_dy_wp, const vector<double> &map_s_wp)
{
    this->map_s_wp = map_s_wp;
    this->map_x_wp = map_x_wp;
    this->map_y_wp = map_y_wp;
    this->map_dx_wp = map_dx_wp;
    this->map_dy_wp = map_dy_wp;
    x_interp.set_points(map_s_wp,  map_x_wp);
    y_interp.set_points(map_s_wp,  map_y_wp);
    dx_interp.set_points(map_s_wp, map_dx_wp);
    dy_interp.set_points(map_s_wp, map_dy_wp);
}


trackmap::trackmap(const string map_file_)
{
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    if (!in_map_.good() || !in_map_.is_open()) {
        cout << "Couldn't read from file '" << map_file_ << "'!" << endl;
    } else {
        string line;
        while (getline(in_map_, line)) {
            istringstream iss(line);
            double x, y;
            double s, d_x, d_y;
            iss >> x;
            iss >> y;
            iss >> s;
            iss >> d_x;
            iss >> d_y;
            map_x_wp.push_back(x);
            map_y_wp.push_back(y);
            map_s_wp.push_back(s);
            map_dx_wp.push_back(d_x);
            map_dy_wp.push_back(d_y);
        }

        x_interp.set_points(map_s_wp,  map_x_wp);
        y_interp.set_points(map_s_wp,  map_y_wp);
        dx_interp.set_points(map_s_wp, map_dx_wp);
        dy_interp.set_points(map_s_wp, map_dy_wp);
    }

    if (in_map_.is_open()) {
        in_map_.close();
    }
}



void trackmap::fit_spline_segment(double car_s)
{
    // get 10 previous and 20 next waypoints
    vector<double> waypoints_segment_x, waypoints_segment_y, waypoints_segment_dx, waypoints_segment_dy;
    vector<int> wp_indices;
    const int lower_wp_i = 9;
    const int upper_wp_i = 20;
    int prev_wp = -1;

    waypoints_segment_s.clear();
    waypoints_segment_s_worldSpace.clear();

    while(car_s > map_s_wp[prev_wp+1] && (prev_wp < (int) (map_s_wp.size()-1) )) {
        prev_wp++;
    }
    for (int i = lower_wp_i; i > 0; i--) {
        if (prev_wp - i < 0)
          wp_indices.push_back(map_s_wp.size() + (prev_wp - i));
        else
          wp_indices.push_back((prev_wp - i) % map_s_wp.size());
    }
    wp_indices.push_back(prev_wp);
    for (int i = 1; i < upper_wp_i; i++) {
        wp_indices.push_back((prev_wp + i) % map_s_wp.size());
    }

    // FILL NEW SEGMENT WAYPOINTS
    const double max_s = 6945.554;
    bool crossed_through_zero = false;
    double seg_start_s = map_s_wp[wp_indices[0]];
    for (int i = 0; i < wp_indices.size(); i++) {
        int cur_wp_i = wp_indices[i];
        waypoints_segment_x.push_back(map_x_wp[cur_wp_i]);
        waypoints_segment_y.push_back(map_y_wp[cur_wp_i]);
        waypoints_segment_dx.push_back(map_dx_wp[cur_wp_i]);
        waypoints_segment_dy.push_back(map_dy_wp[cur_wp_i]);
        // need special treatment of segments that cross over the end/beginning of lap
        if (i > 0) {
            if (cur_wp_i < wp_indices[i-1])
                crossed_through_zero = true;
        }
        waypoints_segment_s_worldSpace.push_back(map_s_wp[cur_wp_i]);
        if (crossed_through_zero) {
            waypoints_segment_s.push_back(abs(seg_start_s - max_s) + map_s_wp[cur_wp_i]);
        } else {
            waypoints_segment_s.push_back(map_s_wp[cur_wp_i] - seg_start_s);
        }
    }
    // fit splines
    x_interp.set_points(waypoints_segment_s, waypoints_segment_x);
    y_interp.set_points(waypoints_segment_s, waypoints_segment_y);
    dx_interp.set_points(waypoints_segment_s, waypoints_segment_dx);
    dy_interp.set_points(waypoints_segment_s, waypoints_segment_dy);
}


// converts world space s coordinate to local space based on provided mapping
double trackmap::get_local_s(double world_s) {
    int prev_wp = 0;
    // special case: first wp in list is larger than s. Meaning we are crossing over 0 somewhere.
    // go to index with value zero first and search from there.
    if (waypoints_segment_s_worldSpace[0] > world_s) {
        while (waypoints_segment_s_worldSpace[prev_wp] != 0.0)
            prev_wp += 1;
    }
    while ((waypoints_segment_s_worldSpace[prev_wp+1] < world_s) &&
           (waypoints_segment_s_worldSpace[prev_wp+1] != 0))  {
        prev_wp += 1;
    }
    double diff_world = world_s - waypoints_segment_s_worldSpace[prev_wp];
    return waypoints_segment_s[prev_wp] + diff_world;
}
     


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> trackmap::getXY(double s, double d) const
{
    int prev_wp = -1;

    while (s > map_s_wp[prev_wp+1] && (prev_wp < (int)(map_s_wp.size()-1) )) {
        prev_wp++;
    }

    int wp2 = (prev_wp+1) % map_x_wp.size();
    double heading = atan2((map_y_wp[wp2]-map_y_wp[prev_wp]), (map_x_wp[wp2]-map_x_wp[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-map_s_wp[prev_wp]);
    double seg_x = map_x_wp[prev_wp] + seg_s*cos(heading);
    double seg_y = map_y_wp[prev_wp] + seg_s*sin(heading);
    double perp_heading = heading-M_PI/2;
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x, y};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> trackmap::getXY_splines(double s, double d) {
    double x = x_interp(s) + dx_interp(s) * d;
    double y = y_interp(s) + dy_interp(s) * d;
    return {x, y};
}

// Transform from Frenet s,d coordinates to Cartesian x,y based on smooth cubic splines
void trackmap::getXYMapAtSD(const vector<double> &s_list, const vector<double> &d_list,
                            vector<double> &map_x_wp, vector<double> &map_y_wp) const
{
    assert(s_list.size() == d_list.size());
    assert(map_x_wp.size() == map_y_wp.size());

    for (int idx = 0; idx < s_list.size(); idx++) {
        //cout << "adding new wp!" << endl;
        map_x_wp.push_back(x_interp(s_list[idx]) + d_list[idx]*dx_interp(s_list[idx]));
        map_y_wp.push_back(y_interp(s_list[idx]) + d_list[idx]*dy_interp(s_list[idx]));
    }
}


double trackmap::getWpX(int idx) const
{
    return map_x_wp[idx];
}

double trackmap::getWpY(int idx) const
{
    return map_y_wp[idx];
}

int trackmap::ClosestWaypoint(double x, double y) const
{
    double closestLen = 1e6; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < map_x_wp.size(); i++)
    {
        double map_x = map_x_wp[i];
        double map_y = map_y_wp[i];
        double dist = distance(x,y,map_x,map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}


int trackmap::NextWaypoint(double x, double y, double theta) const
{
    int closestWaypoint = ClosestWaypoint(x,y);
    double map_x = map_x_wp[closestWaypoint];
    double map_y = map_y_wp[closestWaypoint];
    double heading = atan2((map_y-y),(map_x-x));
    double angle = fabs(theta-heading);
    angle = min(2*M_PI - angle, angle);

    if (angle > M_PI/4)     {
        closestWaypoint++;
        if (closestWaypoint == map_x_wp.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}


double trackmap::distance(double x1, double y1, double x2, double y2) const
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> trackmap::getFrenet(double x, double y, double theta) const
{
    int next_wp = NextWaypoint(x,y, theta);
    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = map_x_wp.size()-1;
    }

    double n_x = map_x_wp[next_wp]-map_x_wp[prev_wp];
    double n_y = map_y_wp[next_wp]-map_y_wp[prev_wp];
    double x_x = x - map_x_wp[prev_wp];
    double x_y = y - map_y_wp[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - map_x_wp[prev_wp];
    double center_y = 2000 - map_y_wp[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(map_x_wp[i], map_y_wp[i], map_x_wp[i+1], map_y_wp[i+1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s, frenet_d};
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> trackmap::getFrenetInterp(double x, double y, double theta) const
{
    cout << "TODO: implement getFrenetInterp()!" << endl;
    return {};
}


void trackmap::getSmoothPath(const vector<vector<double>> &prevPath, vector<vector<double>> &newPath,
                             vector<double> &next_x_vals, vector<double> &next_y_vals, int reusePoints)
{

    double new_x, new_y;  
    vector<double> prev_xy_planned;
    vector<double> xy_planned;
    int prevPathSize = prevPath[0].size();

    // reuse part of previous path if car is starting
    for(int i = 0; i < reusePoints; i++) {
        prev_xy_planned = getXY_splines(newPath[0][i], newPath[1][i]);
        if (prevPathSize > i) {              
            new_x = prevPath[0][i];
            new_y = prevPath[1][i];
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);
        } else {
            next_x_vals.push_back(prev_xy_planned[0]);
            next_y_vals.push_back(prev_xy_planned[1]);
        }
    }

    // assemble rest of the path by adding up deltas in X and Y
    for(int i = reusePoints; i < newPath[0].size(); i++) {
        xy_planned = getXY_splines(newPath[0][i], newPath[1][i]);
        if (prevPathSize > 0) {
            new_x += xy_planned[0] - prev_xy_planned[0];
            new_y += xy_planned[1] - prev_xy_planned[1];
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);                
            prev_xy_planned = xy_planned;
        } else {
            next_x_vals.push_back(xy_planned[0]);
            next_y_vals.push_back(xy_planned[1]);
        }
    }
}