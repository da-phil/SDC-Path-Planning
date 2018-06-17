#include <iostream>
#include <vector>
#include <cmath>
#include "tools.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;



double deg2rad(double x) { return x * pi() / 180.; }
double rad2deg(double x) { return x * 180. / pi(); }


double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int getLane(double d) {
	const double laneWidth = 4.0; // meters
	for (int lane = 0; lane < 4; lane++) {
		if (d > laneWidth*lane && d <= laneWidth*(lane+1)) {
			return lane;
		}
	}
	return 0;
}

int ClosestWaypoint(double x, double y,
	                const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 1e6; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}


int NextWaypoint(double x, double y, double theta,
	             const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2((map_y-y),(map_x-x));
	double angle = fabs(theta-heading);
	angle = min(2*pi() - angle, angle);

	if (angle > pi()/4) 	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size()) {
			closestWaypoint = 0;
		}
	}

	return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
	                     const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if (next_wp == 0) {
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);
	return {frenet_s, frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
	                 const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x, y};
}


// Evaluate a polynomial.
vector<double> polyeval(vector<double> &coeffs, vector<double> &x)
{
	vector<double> result(x.size());
	for (int j = 0; j < x.size(); j++)
	{
		result[j] = 0;
		for (int i = 0; i < coeffs.size(); i++) {
			result[j] += coeffs[i] * pow(x[j], i);
		}
	}
	return result;
}

double polyeval(vector<double> &coeffs, double x)
{
	double result = 0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}
double polyeval(Eigen::VectorXd &coeffs, double x)
{
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

vector<double> polyfit_wp(int wp_start, int wp_stop, int order,
						  vector<double> &map_x, vector<double> &map_y)
{
	assert(map_x.size() == map_y.size());
	int wp_count = wp_stop - wp_start;
	Eigen::VectorXd xvals_eig(wp_count);
	Eigen::VectorXd yvals_eig(wp_count);
	Eigen::MatrixXd A(wp_count, order+1);
	// make sure indicies for map_x and map_y wrap around map size!
	wp_start = wp_start % map_x.size();
	wp_stop  = wp_stop  % map_x.size();

	for (int idx = 0; idx < wp_count; idx++) {
		xvals_eig(idx) = map_x[wp_start+idx];
		yvals_eig(idx) = map_y[wp_start+idx];
	}

	for (int i = 0; i < wp_count; i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < wp_count; j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals_eig(j);
		}
	}
	/*
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	cout << "A: " << endl << A.format(CleanFmt) << endl;
	*/
	auto Q = A.householderQr();
	VectorXd result_eig = Q.solve(yvals_eig);
	vector<double> result(result_eig.data(), result_eig.data() + result_eig.size());
	return result;
}

void polyeval_wp(const vector<double> &coeffs, int wp_start, int wp_stop, int num_points,
                 const vector<double> &map_x, const vector<double> &map_y,
                 vector<double> &wp_interp_x, vector<double> &wp_interp_y)
{
	assert(map_x.size() == map_y.size());
	assert(wp_interp_x.size() == wp_interp_y.size());
	// make sure indicies for map_x and map_y wrap around map size!
	wp_start = wp_start % map_x.size();
	wp_stop  = wp_stop  % map_x.size();
	double dx = 0.5;
	for (int j = 0; j < num_points; j++)
	{
		double y = 0;
		double x = map_x[wp_start] + j*dx;
		for (int i = 0; i < coeffs.size(); i++) {
			y += coeffs[i] * pow(x, i);
		}
		wp_interp_x.push_back(x);
		wp_interp_y.push_back(y);
	}
}
