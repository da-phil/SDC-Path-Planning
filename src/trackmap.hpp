#ifndef TRACKMAP_HPP
#define TRACKMAP_HPP

#include <iostream>
#include <vector>
#include "spline.h"

using namespace std;



class trackmap {
public:
	trackmap(const vector<double> &map_x_wp,  const vector<double> &map_y_wp,
			 const vector<double> &map_dx_wp, const vector<double> &map_dy_wp, const vector<double> &map_s_wp);
	trackmap(const string map_file_);

	//void add_wp(vector<double> map_x, vector<double> map_y) const;

	void getXYMapAtSD(const vector<double> &s_list, const vector<double> &d_list,
					  vector<double> &map_x_wp, vector<double> &map_y_wp) const;
	double getWpX(int idx) const;
	double getWpY(int idx) const;
	int ClosestWaypoint(double x, double y) const;
	int NextWaypoint(double x, double y, double theta) const;
	double distance(double x1, double y1, double x2, double y2) const;

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta) const;
	vector<double> getFrenetInterp(double x, double y, double theta) const;

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d) const;

	~trackmap() {}


//private:

	vector<double> map_s_wp;
	vector<double> map_x_wp;
	vector<double> map_y_wp;
	vector<double> map_dx_wp;
	vector<double> map_dy_wp;

	// interpolate waypoint coordinates with cubic splines
	tk::spline x_interp;
	tk::spline y_interp;
	tk::spline dx_interp;
	tk::spline dy_interp;

	const double max_s = 6945.554;
};


#endif // TRACKMAP_HPP