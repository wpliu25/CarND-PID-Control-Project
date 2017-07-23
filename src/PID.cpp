#include "PID.h"

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
}

void PID::UpdateError(double cte) {

    // update ctes
    prev_cte_ = cte;
    cte_ = cte;
    double diff_cte = cte_ - prev_cte_;
    int_cte_ += cte;

    // update errors
    p_error = Kp_ * cte_;
    i_error = Ki_ * int_cte_;
    d_error = Kd_ * diff_cte;
}

double PID::TotalError() {

    // total_error is the sum of all 3 errors
    return (p_error + i_error + d_error);
}

