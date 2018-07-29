#ifndef POLYNOMIALS_HPP
#define POLYNOMIALS_HPP

#include <vector>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class Polynomial {
public:
    Polynomial() {}
    Polynomial(const Polynomial& orig);
    Polynomial(const vector<double> &coeffs);
    ~Polynomial() {}
    
    void set_coeffs(const vector<double> &coeffs);
    vector<double> get_coeffs() const;

    void polyfit(const Eigen::VectorXd &xvals,
                 const Eigen::VectorXd &yvals,
                 const int order);
    void polyfit(const vector<double> &xvals,
                 const vector<double> &yvals,
                 const int order);

    double polyeval(const double x) const;
    double polyeval_d(const double x) const;
    double polyeval_dd(const double x) const;
    double polyeval_ddd(const double x) const;
    vector<double> polyeval(const vector<double> &x) const;
    vector<double> polyeval_d(const vector<double> &x) const;
    vector<double> polyeval_dd(const vector<double> &x) const;
    vector<double> polyeval_ddd(const vector<double> &x) const;

private:
    double polyeval(const double x,
                    const vector<double> &coeffs) const;
    vector<double> polyeval(const vector<double> &x,
                            const vector<double> &coeffs) const;    
    vector<double> _coeffs;
    vector<double> _coeffs_d;
    vector<double> _coeffs_dd;
    vector<double> _coeffs_ddd;
};

#endif // POLYNOMIALS_HPP

