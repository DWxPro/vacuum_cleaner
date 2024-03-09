#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid.h"

PID::PID( double dt, double max, double min, double Kp, double Ki, double Kd ) :
    dt_(dt),
    max_(max),
    min_(min),
    Kp_(Kp),
    Kd_(Kd),
    Ki_(Ki),
    previous_error_(0),
    integral_(0)
{
}

double PID::calculate( double setpoint, double process_value )
{

    // Calculate error
    double error = setpoint - process_value; 

    // Proportional term
    double Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt_;
    double Iout = Ki_ * integral_;

    // Derivative term
    double derivative = (error - previous_error_) / dt_;
    double Dout = Kd_ * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    previous_error_ = error;

    return output;
}

void PID::setPID(double Kp, double Kd, double Ki)
{
    Kp_ = Kp;
    Kd_ = Kd;
    Ki_ = Ki;
}

#endif