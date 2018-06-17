#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <random>

using namespace std;


class GNB {
public:

  /**
	 * Constructor
	 */
	GNB() {}

  /**
	 * Destructor
	 */
	virtual ~GNB() {}

  double normal_pdf(double x, double m, double s);
  void train(vector<vector<double>> data, vector<string> labels);
	string predict(vector<double>);


private:

  vector<string> possible_labels = {"left","keep","right"};
  map<int, vector<double>> means;
  map<int, vector<double>> vars;
  map<int, double> p_prior; 
};



/* Multivariate gaussian implementation with two variables.
 * @param x  vector containing x-y values
 * @param mu vector of size 2 with means
 * @param std vector of size 2 with standard deviations
 * @output Probability
 */
template<typename T=double>
T multivariate_gaussian(T x[], T mu[], T std[]) {
  T gauss_norm = (1/(2 * M_PI * std[0] * std[1]));
  T exponent   = pow((x[0] - mu[0]), 2)/pow(2 * std[0], 2) + pow((x[1] - mu[1]), 2)/pow(2 * std[1], 2);
  return (gauss_norm * exp(-exponent));
}
#endif // CLASSIFIER_H

