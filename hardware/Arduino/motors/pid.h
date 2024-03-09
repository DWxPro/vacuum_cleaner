#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        // dt -  cycle time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        PID( double dt, double max, double min, double Kp, double Ki, double Kd );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        
        // changes PID parameters
        void setPID( double Kp, double Kd, double Ki );

    private:
        double dt_;
        double max_;
        double min_;
        double Kp_;
        double Kd_;
        double Ki_;
        double previous_error_;
        double integral_;
};

#endif