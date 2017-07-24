#include "PID.h"
#include <limits>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    // initialize tau
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    // initialize errors
    p_error = i_error = d_error = 0.0;

    // initialize ctes
    cte_ = int_cte_ = 0.0;
    prev_cte_ = numeric_limits<double>::max();

}

void PID::UpdateError(double cte) {

    // update ctes
    //  check if it is the initial prev_cte
    if(prev_cte_ == numeric_limits<double>::max())
        prev_cte_ = cte;
    //  proportional
    cte_ = cte;
    //  differential
    double diff_cte = cte_ - prev_cte_;
    prev_cte_ = cte;
    //  integral
    int_cte_ += cte;

    // update errors
    p_error = Kp_ * cte_;
    i_error = Ki_ * int_cte_;
    d_error = Kd_ * diff_cte;
}

double PID::TotalError() {

    // total_error is the sum of all 3 errors
    return -(p_error + i_error + d_error);
}

