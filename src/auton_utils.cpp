#include <iostream>
// #include <ctime>
#include "main.h"
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


    
    //constructer to tell 
    /*
    1. the width of the center of the robot to the left, right, and back encoders
    2. the pros motors you are using (these are passed by pointer)
    3. the encoders you are using (these are also passed by pointer) 
    this makes the program modular:
    */
    AutonUtils::AutonUtils(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor* FL, pros::Motor* FR, pros::Motor* BL, pros::Motor* BR, pros::ADIEncoder* encoderL, pros::ADIEncoder* encoderR, pros::ADIEncoder* encoderM) 
    {
        this->encoder_wheel_radius = encoder_wheel_radius;
        this->wL = wL;
        this->wR = wR;
        this->wM = wM;
        this->FL = FL;
        this->FR = FR;
        this->BL = BL;
        this->BR = BR;
        this->encoderL = encoderL;
        this->encoderR = encoderR;
        this->encoderM = encoderM;
        // globalX = 0;
        // dlX = 0;
        // globalY = 0;
        // dlY = 0;
    }




 
    //method to convert from encoder ticks to wheels for all of the encoder wheels
    //formula (2πr/360) * ticks:

    double AutonUtils::get_left_encoder_distance()
    {
        return ((2 * encoder_wheel_radius * pi) / 360) *  encoderL->get_value();
      
    }

    

    double AutonUtils::get_right_encoder_distance()
    {
        return ((2 * encoder_wheel_radius * pi) / 360) *  encoderR->get_value();
    }

    double AutonUtils::get_middle_encoder_distance()
    {
        return ((2 * encoder_wheel_radius * pi) / 360) * encoderM->get_value();
    }

    //method to compute alpha (should also average it with the imu readings)
    //formula is alpha = (right encoder value reading - left encoder value reading)/width of the robot:

    double AutonUtils::compute_delta_alpha(double delta_right_distance, double delta_left_distance)
    {     
        double delta_alpha = (delta_left_distance - delta_right_distance) / (wL + wR);
        return delta_alpha;
    }

    //method to compute Dly 
    //formula is 2sin(alpha/2) * (right encoder value + (arcR/alpha))

    double AutonUtils::compute_delta_Dly(double delta_alpha, double delta_right_distance)
    {
        if(delta_alpha == 0)
        {
            return delta_right_distance;
        }
        return (2 * sin(delta_alpha / 2)) * (wR + (delta_right_distance /delta_alpha));
    }
    
    //method to comput Dlx
    //formula is 2sin(alpha/2) * (middle tracking wheel value/alpha - wM)

    double AutonUtils::compute_delta_Dlx(double delta_alpha, double delta_middle_distance)
    {
        if(delta_alpha == 0)
        {
            return delta_middle_distance;
        }
        return (2 * sin(delta_alpha / 2)) * ((delta_middle_distance / delta_alpha) + wM);
    }
    
    //method to convert from local variables to global variables
    //formulas are: globalX = (Dlx * cos(alpha/2)) + (Dly * sin(alpha/2)) globalY = (Dly * cos(alpha/2)) - (Dlx * sin(alpha/2)):

    double AutonUtils::compute_delta_globalX(double dlX, double dlY, double delta_alpha)
    {
        return (dlX * cos(delta_alpha/2 + prev_alpha)) + (dlY * sin(delta_alpha/2 + prev_alpha)); 
    }

    double AutonUtils::compute_delta_globalY(double dlX, double dlY, double delta_alpha)
    {
        return (dlY * cos(delta_alpha/2 + prev_alpha)) - (dlX * sin(delta_alpha/2 + prev_alpha)); 
    }


    double AutonUtils::rad_to_deg(double rad)
    {
        double deg = (rad * 180)/pi;
        
        deg = fmod(deg, 360);
        if (deg < 0)
        {
            deg += 360;
        }
        
        return deg;

    }

    //method to convert from degrees to radians
    //formula is 1deg * π/180: 
    double AutonUtils::deg_to_rad(double deg)
    {
        return (deg * pi)/180;
    }

    //method to update functions:
    void AutonUtils::update()
    {   
        double left_encoder_distance = get_left_encoder_distance();
        double delta_left_encoder_distance = left_encoder_distance - prev_left_encoder_distance;
        
        double right_encoder_distance = get_right_encoder_distance();
        double delta_right_encoder_distance = right_encoder_distance - prev_right_encoder_distance;


        double middle_encoder_distance = get_middle_encoder_distance();
        double delta_middle_encoder_distance = middle_encoder_distance - prev_middle_encoder_distance;
        

        double delta_alpha = compute_delta_alpha(delta_right_encoder_distance, delta_left_encoder_distance);
        alpha += delta_alpha;

        double dlX = compute_delta_Dlx(delta_alpha, delta_middle_encoder_distance); 
        double dlY = compute_delta_Dly(delta_alpha, delta_right_encoder_distance); 


        double delta_globalX = compute_delta_globalX(dlX, dlY, delta_alpha); 
        globalX += delta_globalX;

        double delta_globalY = compute_delta_globalY(dlX, dlY, delta_alpha); 
        globalY += delta_globalY;

        prev_left_encoder_distance = left_encoder_distance;
        prev_right_encoder_distance = right_encoder_distance;
        prev_middle_encoder_distance = middle_encoder_distance;
        prev_alpha = alpha;
    }
    
    void AutonUtils::start_update_thread()
    {
        while (true)
        {
            update(); 
            pros::Task::delay(10);
        }
        
    }

    void AutonUtils::make_update_thread()
    {   
        
        // pros::Task update_task([this] {start_update_thread(); });
        task = std::make_shared<pros::Task>([this] { start_update_thread(); });

    }
// class AutonUtils {
//   std::unique_ptr<pros::Task> task{nullptr};


//   void make_update_thread() {
//     task = std::make_shared<pros::Task>([this] { start_update_thread(); });
//   }

//   ~AutonUtils () {
//     if (task.get() != nullptr) {
//       task->remove();
//     }
//   }
// };
    double AutonUtils::compute_alpha(double right_encoder_distance, double left_encoder_distance)
    {
        return (left_encoder_distance - right_encoder_distance) / (wL + wR);
    }

    
    //getter for globalX:

    double AutonUtils::get_globalX()
    {
        return globalX;
    }


    // //getter for globalY

    double AutonUtils::get_globalY()
    {
        return globalY;
    }

    //destructor:

    AutonUtils::~AutonUtils () 
    {
        if (task.get() != nullptr) 
        {
            task->remove();
        }
    }
    
    // //getter for delta_globalY  

    // double AutonUtils::get_delta_globalY()
    // {
    //     return delta_globalY;
    // }

    // //getter for dlX:

    // double AutonUtils::get_dlX()
    // {
    //     return dlX;
    // }

    // //getter for dlY:

    // double AutonUtils::get_dlY()
    // {
    //     return dlY;
    // }

    // double AutonUtils::get_alpha()
    // {
    //    return alpha;
    // }

    double AutonUtils::get_alpha_in_degrees()
    {
       return rad_to_deg(alpha);
    }


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

void point_turn_PID(int target, const double Kp, const double Ki, const double Kd, bool do_once) 
{
    double error, derivative, integral, accumulated_error, change_time, previous_time, previous_error;
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

        previous_error = error;

        pros::delay(20);

    } while ((previous_error > 0.3 && error < .3) || do_once);

    pros::lcd::set_text(1, "pid escaped");
    
}