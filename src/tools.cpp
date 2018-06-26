#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "tools.hpp"


using namespace Eigen;
using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180.; }
double rad2deg(double x) { return x * 180. / pi(); }


double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int getLane(const double d, const double laneWidth) {
	for (int lane = 0; lane <= 2; lane++) {
		if (d > laneWidth*lane && d <= laneWidth*(lane+1)) {
			return lane;
		}
	}
	cout << "couldn't find lane for d=" << d << endl;
	return 0;
}

double getLaneOffsetD(const int lane_number, const double laneWidth) {
	return (laneWidth*lane_number) + 2.0;
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



vector<double> JMT(vector< double> start, vector <double> end, double T)
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
    an array of length 6, each value corresponding to a coefficent in the polynomial 
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

    MatrixXd b = A.inverse() * c;
    
    return {start[0], start[1], 0.5*start[2],
            b(0),     b(1),     b(2)};
}



