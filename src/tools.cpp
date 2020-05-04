#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    Eigen::Vector4d vRmse(0, 0, 0, 0), vTemp(0, 0, 0, 0);

    int iEstimationSize = estimations.size();

    if (iEstimationSize < 1 || iEstimationSize != ground_truth.size())
    {
        std::cout << "Invalid data storage" << std::endl;
        return vRmse;
    }

    for (int i = 0;i < estimations.size();i++) 
    {
        vTemp = estimations[i] - ground_truth[i];
        vTemp = vTemp.array() * vTemp.array();
        vRmse += vTemp;
    }

    vRmse /= iEstimationSize;
    
    vRmse = vRmse.array().sqrt();

    return vRmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3, 4);

    double temp_denominator = pow(x_state[0], 2) + pow(x_state[1], 2);

    VectorXd x = x_state;

    if (temp_denominator < 0.000001)
    {
        std::cout << "Division by Zero" << std::endl;
        return Hj;
    }

    Hj << x[0] / pow(temp_denominator, 0.5), x[1] / pow(temp_denominator, 0.5), 0, 0,
        -x[1] / temp_denominator, x[0] / temp_denominator, 0, 0,
        x[1] * (x[2] * x[1] - x[3] * x[0]) / pow(temp_denominator, 1.5), x[0] * (x[3] * x[0] - x[2] * x[1]) / pow(temp_denominator, 1.5), x[0] / pow(temp_denominator, 0.5), x[1] / pow(temp_denominator, 0.5);

    return Hj;

    
    ////recover state parameters
    //float px = x_state(0);
    //float py = x_state(1);
    //float vx = x_state(2);
    //float vy = x_state(3);

    ////pre-compute a set of terms to avoid repeated calculation
    //float c1 = px * px + py * py;
    //float c2 = sqrt(c1);
    //float c3 = (c1 * c2);

    ////check division by zero
    //if (fabs(c1) < 0.0001) {
    //    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    //    return Hj;
    //}

    ////compute the Jacobian matrix
    //Hj << (px / c2), (py / c2), 0, 0,
    //    -(py / c1), (px / c1), 0, 0,
    //    py* (vx * py - vy * px) / c3, px* (px * vy - py * vx) / c3, px / c2, py / c2;

    //return Hj;


}
