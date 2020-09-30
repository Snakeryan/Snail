#include "PID_controller.h"
#include "main.h"

template <class T>
const T &constrain(const T &x, const T &a, const T &b)
{
    if (x < a)
    {
        return a;
    }
    else if (b < x)
    {
        return b;
    }
    else
        return x;
}

PID_controller::PID_controller(double kP, double kI, double kD, double max_output, double min_output)
{
    this->kP = kP;
    this->kD = kD;
    this->kI = kI;
    this->max_output = max_output;
    this->min_output = min_output;
    for (int i = 0; i < errors_size - 1; i++)
    {
        errors[i] = 0;
    }
}

double PID_controller::compute(double new_error, double current_time)
{

    //assigning the new_error to the data member error:
    error = new_error;

    //assigning the integral value:

    //will set integral back to zero if there is a cross over from positive to negative and use_crossover is true:
    if (((prev_error < 0 && error > 0) || (error < 0 && prev_error > 0)) && use_crossover)
    {
        integral = 0;
    }

    //will set integral only if the error is below a certain amount if use_bounded_error is true:
    if (std::abs(error) < integral_error_limit && use_bounded_error)
    {
        integral += error;
    }
    else if (use_bounded_error)
    {
        integral = 0;
    }
    else
    {
        integral += error;
    }

    //assigning the derivative value
    derivative = prev_error - error;

    //if use_dt time is true, time will be incorporated into derivative and integral
    if (current_time != -1)
    {
        double dt = current_time - prev_time;
        derivative /= dt;
        integral *= dt;
    }

    //assigning the prev variables
    prev_time = current_time;
    prev_error = error;

    //making an error averager

    for (int i = errors_size - 1; i > 0; i--)
    {
        errors[i] = errors[i - 1];
    }
    errors[0] = error;
    //returning the PID value
    double PID_value = error * kP + integral * kI + derivative * kD;

    PID_value = constrain(PID_value, min_output, max_output);

    return PID_value;
}

void PID_controller::use_integrater_error_bound(double integral_error_limit)
{
    this->integral_error_limit = integral_error_limit;
    use_bounded_error = true;
}

void PID_controller::use_crossover_zero()
{
    use_crossover = true;
}

void PID_controller::reset_integral()
{
    integral = 0;
}

double PID_controller::get_error()
{
    return error;
}

double PID_controller::get_integral()
{
    return integral;
}

double PID_controller::get_derivative()
{
    return derivative;
}

double PID_controller::get_error_average(int errors_to_average)
{
    double error_sum = 0;
    for (int i = 0; i < errors_to_average; i++)
    {
        error_sum += errors[i];
    }
    return error_sum / errors_to_average;
}