#include "polynomials.hpp"

Polynomial::Polynomial(const Polynomial& src) {
  _coeffs = src._coeffs;
  _coeffs_d = src._coeffs_d;
  _coeffs_dd = src._coeffs_dd;
  _coeffs_ddd = src._coeffs_ddd;
}


Polynomial::Polynomial(const vector<double> &coeffs) {
  set_coeffs(coeffs);
}


void Polynomial::set_coeffs(const vector<double> &coeffs) {
  for (int i = 0; i < coeffs.size(); i++)
  {
    _coeffs.push_back(coeffs[i]);

    double d = i * coeffs[i];
    if (i > 0)
      _coeffs_d.push_back(d);
    if (i > 1)
      _coeffs_dd.push_back((i - 1) * d);
    if (i > 2)
      _coeffs_ddd.push_back((i - 2) * d);  
  }    
}


vector<double> Polynomial::get_coeffs() const {
  return _coeffs;
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
void Polynomial::polyfit(const Eigen::VectorXd &xvals,
                         const Eigen::VectorXd &yvals,
                         const int order)
{
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
  Eigen::VectorXd result_eig = Q.solve(yvals);
  vector<double> result(result_eig.data(), result_eig.data() + result_eig.size());
  set_coeffs(result);
}

void Polynomial::polyfit(const vector<double> &xvals,
                         const vector<double> &yvals,
                         const int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::VectorXd xvals_eig = Eigen::VectorXd::Map(xvals.data(), xvals.size());
  Eigen::VectorXd yvals_eig = Eigen::VectorXd::Map(yvals.data(), yvals.size());
  polyfit(xvals_eig, yvals_eig, order);
}


double Polynomial::polyeval(const double x,
                            const vector<double> &coeffs) const
{
  double result = 0;
  for (int i = 0; i < coeffs.size(); i++) {
     result += coeffs[i] * pow(x, i);
  }
  return result;
}

vector<double> Polynomial::polyeval(const vector<double> &x,
                                    const vector<double> &coeffs) const
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


double Polynomial::polyeval(const double x) const {
  return polyeval(x, _coeffs);
}

double Polynomial::polyeval_d(const double x) const {
  return polyeval(x, _coeffs_d);
}

double Polynomial::polyeval_dd(const double x) const {
  return polyeval(x, _coeffs_dd);   
}

double Polynomial::polyeval_ddd(const double x) const {
  return polyeval(x, _coeffs_ddd);
}


vector<double> Polynomial::polyeval(const vector<double> &x) const {
  return polyeval(x, _coeffs);
}

vector<double> Polynomial::polyeval_d(const vector<double> &x) const {
  return polyeval(x, _coeffs_d);
}

vector<double> Polynomial::polyeval_dd(const vector<double> &x) const {
  return polyeval(x, _coeffs_dd);   
}

vector<double> Polynomial::polyeval_ddd(const vector<double> &x) const {
  return polyeval(x, _coeffs_ddd);
}
