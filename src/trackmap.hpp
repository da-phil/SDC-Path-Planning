#ifndef TRACKMAP_HPP
#define TRACKMAP_HPP

#include <iostream>
#include <vector>
#include "spline.h"
#include "tools.hpp"

using namespace std;



typedef	struct {
	vector<double> x;
	vector<double> y;	
	vector<double> s;
	vector<double> d;
	vector<double> v;

	// interpolate y in respect to x
	void smoothXY() {
		// first smooth s, then x and y with respect to s
		vector<double> range(s.size());
		for (int i = 0; i < s.size(); i++)
			range[i] += i;
		auto coeffs = polyfit(s, x, 5);
		x = polyeval(coeffs, s);
		coeffs = polyfit(s, y, 5);
		y = polyeval(coeffs, s);
		// and then smooth again y with respect to x
		coeffs = polyfit(x, y, 5);
		y = polyeval(coeffs, x);

		/*
		cout << "path2 = np.array([";
		for (int i = 0; i < x.size(); i++)
		{
			cout << "[" << x[i] << ", " << y[i] << "]," << endl;
		}
		cout << "])" << endl;
		*/
	}
	void clear() {
		x.clear();
		y.clear();
		s.clear();
		d.clear();
		v.clear();
	}

	void removeFirstPoints(int numPoints) {
		if (x.size() < numPoints)
			return;
		
		x.erase(x.begin(), x.begin() + numPoints);
		y.erase(y.begin(), y.begin() + numPoints);
		s.erase(s.begin(), s.begin() + numPoints);
		d.erase(d.begin(), d.begin() + numPoints);
		v.erase(v.begin(), v.begin() + numPoints);
	}

	void removeLastPoints(int numPoints) {
		if (x.size() < numPoints)
			return;

		x.erase(x.end() - numPoints, x.end());
		y.erase(y.end() - numPoints, y.end());
		s.erase(s.end() - numPoints, s.end());
		d.erase(d.end() - numPoints, d.end());
		v.erase(v.end() - numPoints, v.end());
	}

	int size(void) const {
		return x.size();
	}
	
} path_t;


class trackmap {
public:
	trackmap(const vector<double> &map_x_wp,  const vector<double> &map_y_wp,
			 const vector<double> &map_dx_wp, const vector<double> &map_dy_wp, const vector<double> &map_s_wp);
	trackmap(const string map_file_);

	void getXYMapAtSD(const vector<double> &s_list, const vector<double> &d_list,
					  vector<double> &map_x_wp, vector<double> &map_y_wp) const;
	void getXYMapAtSD(const vector<double> &s_list, const vector<double> &d_list,
                      path_t &path, const int reuse_wp_cnt) const;
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