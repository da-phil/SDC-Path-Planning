#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include "spline.h"

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

	if (angle > M_PI/4) 	{
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
