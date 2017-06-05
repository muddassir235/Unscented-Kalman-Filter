#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// checking the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size() == 0){
	    std::cout << "Error: No Estimations Given";
	    return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()){
	    std::cout << "Error: Number of Estimations and Ground Truth values should be equal.";
	    return rmse;
	}
	
	//accumulating squared residuals
	for(int i=0; i < estimations.size(); i++){
        // ... your code here
        VectorXd diff = estimations[i] - ground_truth[i];
		VectorXd curr_square = diff.array()*diff.array();
		rmse += curr_square;
	}

	//calculate the mean
	// ... your code here
	rmse /= estimations.size();

	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
    
	//return the result
	return rmse;
}