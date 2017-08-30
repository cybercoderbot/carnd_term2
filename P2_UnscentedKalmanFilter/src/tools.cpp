#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Accumulate squared errors
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  // Compute the square root of mean squared error
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
