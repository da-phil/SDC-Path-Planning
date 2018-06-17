#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include <Eigen/Dense>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int getLane(double d);

int ClosestWaypoint(double x, double y,
	                const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
	             const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
	                     const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
	                 const vector<double> &maps_x, const vector<double> &maps_y);

vector<double> polyeval(vector<double> &coeffs, vector<double> &x);
double polyeval(vector<double> &coeffs, double x);
double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);


vector<double> polyfit_wp(int wp_start, int wp_stop, int order,
						  vector<double> &map_x, vector<double> &map_y);
void polyeval_wp(const vector<double> &coeffs, int wp_start, int wp_stop, int num_points,
                 const vector<double> &map_x, const vector<double> &map_y,
                 vector<double> &wp_interp_x, vector<double> &wp_interp_y);

 //define TOOLS_HPP
 #endif