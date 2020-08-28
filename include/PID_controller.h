#ifndef PID_H
#define PID_H

class PID_controller
{
    double error, kP, integral, kI, derivative, kD, max_output, min_output, prev_error, prev_time, time, integral_error_limit = 0;

    bool use_crossover, use_bounded_error = false;

public:
    PID_controller(double kP, double kI, double kD, double max_output, double min_output);

    double compute(double newError, bool use_dt = false);

    void use_integrater_error_bound(double integral_error_limit);

    void use_crossover_zero();

    void reset_integral();

    double get_prev_time_since_PID_initialized();

    double get_time_since_PID_initialized();

    double get_error();

    double get_integral();

    double get_derivative();

};

#endif