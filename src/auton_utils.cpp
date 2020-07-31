#include <iostream>
// #include <ctime>
#include "auton.h"
#include "globals.h"
//must have:
/*
turn function that makes the robot's motors turn
PID loop function that has arguments inputs that are the k constants and returns the value of the PID 
*/

// PID function
/*
this function should have error, derivative, and integral variables
the error variable should equal the proportional control or:    the target angle - the current angle
the derivative variable should eqaul the slope of the time vs error graph or: 
previous error - current error/ previous time - current time
the integral variable should equal the area of the time vs error graph or
previous integral value + ∆t * error
the function will also run the turn function and have its input as:
turn(error * Kp + derivative * Kd + integral * Ki)
*/

//path to take:
/*
I want the robot to turn in the optimal direction
for example, if the ange is 90 degrees from 0 degrees you would want the robot to turn 90 degrees and not 270 degrees
so, if the degrees < 180, then turn negative degrees, else turn positive degrees
*/

//optionals:
/*
have a vector that keeps track of all the errors and can be accessed
*/


void set_turn(int turn)
{
    FL = turn;
    FR = turn;
    BR = turn;
    BL = turn;
}

// this funcction is more complex than the turn function

double compute_error(double target, double current_angle)
{
    double error = current_angle - target;
    if(error < -180)
    {
        return -(error + 360);
    }
    else if(error >= -180 && error < 1) 
    {
        return abs(error);
    }    
    else if(error > 0 && error < 180)
    {
        return -error;
    } 
    else
    {
       return (error - 360);
    }
}

void point_turn_PID(int target, const double Kp, const double Ki, const double Kd) 
{
    double error, derivative, integral, time, accumulated_error, change_time, previous_time, previous_error = 0;
    do
    {
        error = compute_error(target, IMU.get_heading());

        

        derivative = (previous_error - error);
        
        if(abs(error) < 25) 
        {
            integral += error;
        }    
        else 
        {
            integral = 0;
        }

        if((previous_error < 0 && error > 0) || (error < 0 && previous_error > 0)) integral = 0;
        
        
        set_turn(error*Kp + integral*Ki + derivative*Kd);

        

        pros::lcd::set_text(6, "the error is: " + std::to_string(error));
        pros::lcd::set_text(7, "the target is: " + std::to_string(target));
        pros::lcd::set_text(4, "the derivative is: " + std::to_string(derivative));
        pros::lcd::set_text(3, "the integral is: " + std::to_string(accumulated_error));
        pros::lcd::set_text(5, "PID value is: " + std::to_string(error*Kp + integral*Ki + derivative*Kd));
        pros::lcd::set_text(2, "the imu value is: " + std::to_string(IMU.get_heading()));
        pros::lcd::set_text(1, "the time is: " + std::to_string(time));

        previous_error = error;

        pros::delay(20);

    } while (true);

    pros::lcd::set_text(1, "pid escaped");
    
}