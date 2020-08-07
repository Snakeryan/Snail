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

//motion profiling functions:
/*
    functions to calculate the speed to turn the motors:

    FL = P2/s(1 - abs(R)) + R * S

    FR = P1/s(1 - abs(R)) - R * S

    BL = P1/s(1 - abs(R)) + R * S

    BR = P2/s(1 - abs(R)) - R * S
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


double AutonUtils::rad_to_deg_wraped(double rad)
{
    double deg = (rad * 180)/pi;
    
    deg = fmod(deg, 360);
    if (deg < 0)
    {
        deg += 360;
    }
    
    return deg;

}

double AutonUtils::rad_to_deg(double rad)
{
    return (rad * 180)/pi;
}


double AutonUtils::get_alpha_in_degrees()
{
    return rad_to_deg_wraped(alpha);
}


//method to convert from degrees to radians
//formula is 1deg * π/180: 
double AutonUtils::deg_to_rad(double deg)
{
    return (deg * pi)/180;
}

void AutonUtils::turn_to_point(double destX, double destY)
{
    double angle = atan2(destX - globalX, destY - globalY);
    // pros::lcd::set_text(0, "the angle to turn is: " + std::to_string(rad_to_deg(angle)));
    if(angle < 0)
    {
        angle += TAU;
    }
    pros::lcd::set_text(0, "the angle to turn is: " + std::to_string(rad_to_deg_wraped(angle)));
    point_turn_PID(angle);
}

//method to update functions:
void AutonUtils::update()
{ 
    update_mutex.take(1000);  
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
    update_mutex.give();
}

double AutonUtils::get_constrained_alpha()
{
    double theta = alpha;
    theta = fmod(theta, TAU);
    if (theta < 0)
    {
        theta += TAU;
    }
    return theta;

}

/*
functions to calculate main variable:

T = arctan2(dlY, dlX)

S = MIN(sqrt(pow(dlX, 2) + pow(dlY, 2)), 1)

*/






/*
functions to calculate intermediate steps:

P1 = -cos(T + pi/4)

P2 = sin(T + pi/4)

s = max(abs(P1), abs(P2)) / S
*/

double AutonUtils::compute_P1(double T)
{
    return -cos(T + pi/4);
}

double AutonUtils::compute_P2(double T)
{
    return sin(T + pi/4);
}

double AutonUtils::compute_s(double P1, double P2, double S)
{
    return MAX(abs(P1), abs(P2)) / S;
}

/*
    functions to calculate the speed to turn the motors:

    FL = P2/s(1 - abs(R)) + R * S

    FR = P1/s(1 - abs(R)) - R * S

    BL = P1/s(1 - abs(R)) + R * S
    
    BR = P2/s(1 - abs(R)) - R * S
*/

void AutonUtils::compute_FL_motor_speed(double P2, double s, double K_constant, double R, double use_motor)
{
    double FL_speed = (P2 / s) * (1 - abs(R)) + R * K_constant;
    pros::lcd::set_text(1, "FL motor speed: " + std::to_string(FL_speed));
    FL-> move_voltage(FL_speed * 12700 * use_motor);
}

void AutonUtils::compute_FR_motor_speed(double P1, double s, double K_constant, double R, double use_motor)
{
    double FR_speed = ((P1 / s) * (1 - abs(R)) - R * K_constant) * -1;
    pros::lcd::set_text(2, "FR motor speed: " + std::to_string(FR_speed));
    FR-> move_voltage(FR_speed * 12700 * use_motor);
}

void AutonUtils::compute_BL_motor_speed(double P1, double s, double K_constant, double R, double use_motor)
{
    double BL_speed = ((P1 / s) * (1 - abs(R)) + R * K_constant); // was multiplied by -1
    pros::lcd::set_text(3, "BL motor speed: " + std::to_string(BL_speed));
    BL-> move_voltage(BL_speed * 12700 * use_motor);
}

void AutonUtils::compute_BR_motor_speed(double P2, double s, double K_constant, double R, double use_motor)
{
    double BR_speed = ((P2 / s) * (1 - abs(R)) - R * K_constant) * -1; 
    pros::lcd::set_text(4, "BR motor speed: " + std::to_string(BR_speed));
    BR-> move_voltage(BR_speed * 127000 * use_motor);
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

void AutonUtils::set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees)
{
    update_mutex.take(1000);
    globalX = new_X;
    globalY = new_Y;
    prev_alpha = deg_to_rad(new_alpha_in_degrees);
    alpha = prev_alpha;
    update_mutex.give();
}
void AutonUtils::set_turn(int turn)
{   
    FL-> move_voltage(turn * 1000);
    FR-> move_voltage(turn * 1000);
    BR-> move_voltage(turn * 1000);
    BL-> move_voltage(turn * 1000);
}

//function to call all of the motion algorithms and update the variables:

void AutonUtils::drive_to_point(double tX, double tY, double target_angle_in_degrees)
{
    double initial_distance_error = sqrt(pow(tX - globalX, 2) + pow(tY - globalY, 2));
    double target_angle = deg_to_rad(target_angle_in_degrees);
    while(true)
    {
        double error_in_X = tX - globalX;
        double error_in_Y = tY - globalY; 
        
        double angle_error = compute_error(target_angle, get_constrained_alpha());

        double arc_length_error = angle_error * wM;
        
        double current_distance_error = sqrt(pow(error_in_X, 2) + pow(error_in_Y, 2));

         

        double R =  MIN(arc_length_error / (10 + initial_distance_error), 1); // formula was previously: 0.5 * arc_length_error / (current_distance_error + arc_length_error * 0.5);

        R = MIN(1, R);
        R = MAX(-1, R);

        double S = MIN(current_distance_error, 1); // MIN((current_distance_error / initial_distance_error) * 2, 1);
    
        double T = atan2(error_in_Y, error_in_X) + alpha;
        
        double P1 = compute_P1(T);
        
        double P2 = compute_P2(T);
        
        double s = compute_s(P1, P2, S);

        double motors_on_off = 1;

        double K_constant = 1;
        
        compute_FL_motor_speed(P2, s, K_constant, R, motors_on_off);
        
        compute_FR_motor_speed(P1, s, K_constant, R, motors_on_off);
        
        compute_BL_motor_speed(P1, s, K_constant, R, motors_on_off);
        
        compute_BR_motor_speed(P2, s, K_constant, R, motors_on_off);

        pros::lcd::set_text(5, "the error angle: " + std::to_string(rad_to_deg(angle_error)));
       
        pros::lcd::set_text(6, "target angle:  " + std::to_string(rad_to_deg(target_angle)));
        pros::lcd::set_text(7, "constrained alpha: " + std::to_string(rad_to_deg(get_constrained_alpha())));
        pros::delay(20);
    } 
}

void AutonUtils::point_turn_PID(double target, const double Kp, const double Ki, const double Kd, bool use_IMU, bool do_once) 
{
    double error, derivative, integral, accumulated_error, change_time, previous_time, previous_error;
    do
    {
        if (use_IMU)
        {
            error = compute_error(target, deg_to_rad(IMU.get_heading()));
        }
        else
        {
            error = compute_error(target, get_constrained_alpha());
        }

        derivative = (previous_error - error);
        
        if(abs(error) < 0.349066) // 20 degrees
        {
            integral += error;
        }    
        else 
        {
            integral = 0;
        }

        if((previous_error < 0 && error > 0) || (error < 0 && previous_error > 0)) integral = 0;
        
        
        set_turn(error*Kp + integral*Ki + derivative*Kd);

        pros::lcd::set_text(6, "the error is: " + std::to_string(error * 180 / pi));
        pros::lcd::set_text(7, "the target is: " + std::to_string(rad_to_deg_wraped(target)));
        pros::lcd::set_text(4, "the derivative is: " + std::to_string(derivative));
        pros::lcd::set_text(3, "the integral is: " + std::to_string(accumulated_error));
        pros::lcd::set_text(5, "PID value is: " + std::to_string(error*Kp + integral*Ki + derivative*Kd));
        pros::lcd::set_text(2, "alpha is: " + std::to_string(get_alpha_in_degrees()));

        previous_error = error;

        pros::delay(20);

    } while (true || do_once);//previous_error > 0.3 && error < .3

    pros::lcd::set_text(1, "pid escaped");
    
}


double AutonUtils::compute_error(double target, double current_angle)
{
    double error = current_angle - target;
    if(error < -pi)
    {
        return -(error + TAU);
    }
    else if(error >= -pi && error <= 0) 
    {
        return abs(error);
    }    
    else if(error > 0 && error < pi)
    {
        return -error;
    } 
    else
    {
    return (error - TAU);
    }
    
    // double error;

    // if (abs(target - current_angle) < abs(target - TAU - current_angle))
    // {
    //     error = target - current_angle; // No zero cross
    // }
    
    // else if (error > pi)
    // {
    //     error = error -TAU;
    // }

    // else if (error < -pi)
    // {
    //     error = error + TAU; 
    // }
    // else
    // {
    //     error = target - TAU - current_angle; // Zero cross
    // }
        

    
    return error;
}


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
    