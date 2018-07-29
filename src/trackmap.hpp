#ifndef TRACKMAP_HPP
#define TRACKMAP_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include "spline.h"
#include "tools.hpp"

using namespace std;



class trackmap {

public:
    trackmap(const vector<double> &map_x_wp,  const vector<double> &map_y_wp,
             const vector<double> &map_dx_wp, const vector<double> &map_dy_wp, const vector<double> &map_s_wp);
    trackmap(const string map_file_);
    ~trackmap() {}

    double getWpX(int idx) const;
    double getWpY(int idx) const;
    int ClosestWaypoint(double x, double y) const;
    int NextWaypoint(double x, double y, double theta) const;
    double distance(double x1, double y1, double x2, double y2) const;

    void getSmoothPath(const vector<vector<double>> &prevPath, vector<vector<double>> &newPath,
                       vector<double> &next_x_vals, vector<double> &next_y_vals, int reusePoints = 5);
    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d) const;
    vector<double> getXY_splines(double s, double d);

    void getXYMapAtSD(const vector<double> &s_list, const vector<double> &d_list,
                      vector<double> &map_x_wp, vector<double> &map_y_wp) const;

    double get_local_s(double world_s);    
    void fit_spline_segment(double car_s);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    vector<double> getFrenet(double x, double y, double theta) const;
    vector<double> getFrenetInterp(double x, double y, double theta) const;


    vector<double> map_s_wp;
    vector<double> map_x_wp;
    vector<double> map_y_wp;
    vector<double> map_dx_wp;
    vector<double> map_dy_wp;

    // Cubic splines representing interpolated waypoint coordinates
    tk::spline x_interp;
    tk::spline y_interp;
    tk::spline dx_interp;
    tk::spline dy_interp;

    vector<double> waypoints_segment_s_worldSpace;
    vector<double> waypoints_segment_s;

    // The max s value before wrapping around the track back to 0
    const double max_s = 6945.554;
};


#endif // TRACKMAP_HPP