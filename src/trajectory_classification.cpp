#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <cstring>

#include "trajectory_classification.hpp"


using namespace std;



double GNB::normal_pdf(double x, double m, double s)
{
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (x - m) / s;
    return inv_sqrt_2pi / s * std::exp(-0.5f * a*a);
}


void GNB::train(vector<vector<double>> data, vector<string> labels)
{
	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

	int sample_cnt = data.size();
	int feature_cnt = data[0].size();
	int class_cnt = possible_labels.size();
	
	double sum[feature_cnt];
    map<int, vector<int>> class_indexes;

    // find sample indexes which belong to one class / label
    for (int class_idx = 0; class_idx < class_cnt; class_idx++) {
        vector<int> sample_indexes;
        for(int i = 0; i < sample_cnt ; i++) {
            if (labels[i] == possible_labels[class_idx]) {
                sample_indexes.push_back(i);
            }
        }
        class_indexes.insert(make_pair(class_idx, sample_indexes));
    }
    
    means.clear();
    vars.clear();
    for (int class_idx = 0; class_idx < class_cnt; class_idx++) {
        cout << "==================================================" << endl;
        cout << "stats for class " << possible_labels[class_idx] << endl;
        
        // calc prior probablity for class
        p_prior.insert(make_pair(class_idx,
                                 class_indexes[class_idx].size() / (double) sample_cnt));
        cout << "prior probability: " << p_prior[class_idx] << endl;
        
        
        // calc means for class / label
        memset(sum, 0, sizeof(sum));
        vector<double> means_;
        cout << "means : ";
        for (int j = 0; j < feature_cnt; j++) {    
            for(auto &i: class_indexes[class_idx]) {
                sum[j] += data[i][j]; // sum up individual feature values
            }
            means_.push_back(sum[j] / sample_cnt);
            cout << means_[j] << ", ";
        }
        means.insert(make_pair(class_idx, means_));
        cout << endl;

        // calc variances for class / label
        memset(sum, 0, sizeof(sum));
        vector<double> vars_;
        cout << "variances: ";
        for (int j = 0; j < feature_cnt; j++) {
            for(auto &i: class_indexes[class_idx]) {
                sum[j] += pow(data[i][j] - means_[j], 2);
            }
            vars_.push_back(sum[j] / sample_cnt);
            cout << vars_[j] << ", ";
        }
        vars.insert(make_pair(class_idx, vars_));
        cout << endl;
    }
}


string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
	*/
 
    map<string,double > results;
    double total_prob = 0;
    double highest_score = 0;
    int highest_score_label_idx = 0;
    
    for (int class_idx = 0; class_idx < possible_labels.size(); class_idx++) {
        double result = p_prior[class_idx]; // start with prior probability for class
        for (int feature_idx = 0; feature_idx < sample.size(); feature_idx++) {
            result *= normal_pdf(sample[feature_idx],
                                 means[class_idx][feature_idx],
                                 sqrt(vars[class_idx][feature_idx]));
        }
        total_prob += result;
        if (result > highest_score) {
            highest_score = result;
            highest_score_label_idx = class_idx;
        }
        results.insert(make_pair(possible_labels[class_idx], result));
    }
    
    // normalize probablities
    for (auto &result: results) {
        result.second /= total_prob;
        //cout << "result for class '" << result.first << "': " << result.second << endl;        
    }
    
	return possible_labels[highest_score_label_idx];
}

