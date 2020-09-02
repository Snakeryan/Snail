#ifndef PID_H
#define PID_H

class PID_controller
{
    double error, kP, integral, kI, derivative, kD, max_output, min_output, prev_error, prev_time, time, integral_error_limit = 0;

    double errors[20];
    const int errors_size = 20;
    //flags:
    bool use_crossover, use_bounded_error = false;

public:
    /**
 *        all the constant variables should be positive
 * \param kP
 *        constant value that proportional control is multiplied by 
 * \param kI
 *        constant value that integral control is multiplied by
 * \param kD
 *        constant value that derivative control is multiplied by
 * \param max_output
 *        the maximum value that the PID value can output
 * \param min_output
 *        the minumum value that the PID value can output
*/
    PID_controller(double kP, double kI, double kD, double max_output, double min_output);

    /**
 * \param newError
 *        the current error that the system has 
 * \param current_time
 *        if you want to use time within the PID algorithms, pass in the time (-1 is the default parameter for no time)
 * \return
 *        the PID value to be assigned to the controller
*/
    double compute(double newError, double current_time = -1);

    /**
 *        adds integral when the error value is below an inputed amount
 * \param integral_error_limit
 *        the value that the PID controller will start adding integral
*/
    void use_integrater_error_bound(double integral_error_limit);

    /**
 *        will put the integral value back to zero when the error goes from negative to positive or vice-versa
*/
    void use_crossover_zero();

    /**
 *        resets the integral value to zero
*/
    void reset_integral();

    /**
 * \return
 *        the current error in the PID controller
*/
    double get_error();

    /**
 * \return
 *        the current integral value
*/
    double get_integral();

    /**
 * \return
 *        the current derivative value
*/
    double get_derivative();

    double get_error_average(int errors_to_average);
};

#endif