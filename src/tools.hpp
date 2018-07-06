#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <vector>
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;


double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);
double norm(double x, double y);
double mph2mps(double mph);
double mps2mph(double mps);

int getLane(const double d, const double laneWidth=4.0);
double getLaneOffsetD(const int lane_number, const double laneWidth=4.0);

vector<double> polyfit(vector<double> &xvals, vector<double> &yvals, int order);
Eigen::VectorXd polyfit(Eigen::VectorXd &xvals, Eigen::VectorXd &yvals, int order);

vector<double> polyeval(vector<double> &coeffs, vector<double> &x);
double polyeval(vector<double> &coeffs, double x);
double polyeval(Eigen::VectorXd coeffs, double x);

vector<double> polyfit_wp(int wp_start, int wp_stop, int order,
						  vector<double> &map_x, vector<double> &map_y);

vector<double> JMT(vector< double> start, vector <double> end, double T);

 //define TOOLS_HPP
 #endif