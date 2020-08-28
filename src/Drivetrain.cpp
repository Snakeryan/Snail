#include <iostream>
#include "main.h"
#include "auton.h"
#include "globals.h"
#include "PID_controller.h"

//motion profiling functions:
/*
    functions to calculate the speed to turn the motors:

    FL = P2/s(1 - abs(R)) + R * S

    FR = P1/s(1 - abs(R)) - R * S

    BL = P1/s(1 - abs(R)) + R * S

    BR = P2/s(1 - abs(R)) - R * S
*/

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

DriveTrain::DriveTrain(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor *FL, pros::Motor *FR, pros::Motor *BL, pros::Motor *BR, pros::ADIEncoder *encoderL, pros::ADIEncoder *encoderR, pros::ADIEncoder *encoderM)
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
}

double DriveTrain::compute_alpha(double right_encoder_distance, double left_encoder_distance)
{
    return (left_encoder_distance - right_encoder_distance) / (wL + wR);
}

//method to convert from encoder ticks to wheels for all of the encoder wheels
//formula (2πr/360) * ticks:

double DriveTrain::get_left_encoder_distance()
{
    return ((2 * encoder_wheel_radius * pi) / 360) * encoderL->get_value();
}

double DriveTrain::get_right_encoder_distance()
{
    return ((2 * encoder_wheel_radius * pi) / 360) * encoderR->get_value();
}

double DriveTrain::get_middle_encoder_distance()
{
    return ((2 * encoder_wheel_radius * pi) / 360) * encoderM->get_value();
}

//method to compute alpha (should also average it with the imu readings)
//formula is alpha = (right encoder value reading - left encoder value reading)/width of the robot:

double DriveTrain::compute_delta_alpha(double delta_right_distance, double delta_left_distance)
{
    double delta_alpha = (delta_left_distance - delta_right_distance) / (wL + wR);
    return delta_alpha;
}

//method to compute Dly
//formula is 2sin(alpha/2) * (right encoder value + (arcR/alpha))

double DriveTrain::compute_delta_Dly(double delta_alpha, double delta_right_distance)
{
    if (delta_alpha == 0)
    {
        return delta_right_distance;
    }
    return (2 * sin(delta_alpha / 2)) * (wR + (delta_right_distance / delta_alpha));
}

//method to comput Dlx
//formula is 2sin(alpha/2) * (middle tracking wheel value/alpha - wM)

double DriveTrain::compute_delta_Dlx(double delta_alpha, double delta_middle_distance)
{
    if (delta_alpha == 0)
    {
        return delta_middle_distance;
    }
    return (2 * sin(delta_alpha / 2)) * ((delta_middle_distance / delta_alpha) + wM);
}

//method to convert from local variables to global variables
//formulas are: globalX = (Dlx * cos(alpha/2)) + (Dly * sin(alpha/2)) globalY = (Dly * cos(alpha/2)) - (Dlx * sin(alpha/2)):

double DriveTrain::compute_delta_globalX(double Dlx, double Dly, double delta_alpha)
{
    return (Dlx * cos(delta_alpha / 2 + prev_alpha)) + (Dly * sin(delta_alpha / 2 + prev_alpha));
}

double DriveTrain::compute_delta_globalY(double Dlx, double Dly, double delta_alpha)
{
    return (Dly * cos(delta_alpha / 2 + prev_alpha)) - (Dlx * sin(delta_alpha / 2 + prev_alpha));
}

double DriveTrain::convert_rad_to_deg_wraped(double rad)
{
    double deg = (rad * 180) / pi;

    deg = fmod(deg, 360);
    if (deg < 0)
    {
        deg += 360;
    }

    return deg;
}

double DriveTrain::convert_rad_to_deg(double rad)
{
    return (rad * 180) / pi;
}

double DriveTrain::get_alpha_in_degrees()
{
    return convert_rad_to_deg_wraped(alpha);
}

//method to convert from degrees to radians
//formula is 1deg * π/180:
double DriveTrain::convert_deg_to_rad(double deg)
{
    return (deg * pi) / 180;
}

//method to update functions:
void DriveTrain::update_odometry()
{
    update_odometry_mutex.take(1000);
    double left_encoder_distance = get_left_encoder_distance();
    double delta_left_encoder_distance = left_encoder_distance - prev_left_encoder_distance;

    double right_encoder_distance = get_right_encoder_distance();
    double delta_right_encoder_distance = right_encoder_distance - prev_right_encoder_distance;

    double middle_encoder_distance = get_middle_encoder_distance();
    double delta_middle_encoder_distance = middle_encoder_distance - prev_middle_encoder_distance;

    double delta_alpha = compute_delta_alpha(delta_right_encoder_distance, delta_left_encoder_distance);
    alpha += delta_alpha;

    double Dlx = compute_delta_Dlx(delta_alpha, delta_middle_encoder_distance);
    double Dly = compute_delta_Dly(delta_alpha, delta_right_encoder_distance);

    double delta_globalX = compute_delta_globalX(Dlx, Dly, delta_alpha);
    globalX += delta_globalX;

    double delta_globalY = compute_delta_globalY(Dlx, Dly, delta_alpha);
    globalY += delta_globalY;

    prev_left_encoder_distance = left_encoder_distance;
    prev_right_encoder_distance = right_encoder_distance;
    prev_middle_encoder_distance = middle_encoder_distance;
    prev_alpha = alpha;
    update_odometry_mutex.give();
}

void DriveTrain::set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees)
{
    update_odometry_mutex.take(1000);
    globalX = new_X;
    globalY = new_Y;
    prev_alpha = convert_deg_to_rad(new_alpha_in_degrees);
    alpha = prev_alpha;
    update_odometry_mutex.give();
}

void DriveTrain::turn_to_point(double destX, double destY)
{
    double angle = atan2(destX - globalX, destY - globalY);
    if (angle < 0)
    {
        angle += TAU;
    }
    pros::lcd::set_text(0, "the angle to turn is: " + std::to_string(convert_rad_to_deg_wraped(angle)));
    point_turn_PID(angle, 30, .7, -60);
}

/*
functions to calculate intermediate steps:

P1 = -cos(T + pi/4)

P2 = sin(T + pi/4)

s = max(abs(P1), abs(P2)) / S
*/

double DriveTrain::compute_P1(double T)
{
    return -cos(T + pi / 4);
}

double DriveTrain::compute_P2(double T)
{
    return sin(T + pi / 4);
}

double DriveTrain::compute_s(double P1, double P2, double S)
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

void DriveTrain::compute_FL_motor_speed(double P2, double s, double K_constant, double R, double multiplier)
{
    double FL_speed = (P2 / s) * (1 - abs(R)) + R * K_constant;
    FL->move_voltage(FL_speed * 12700 * multiplier);
}

void DriveTrain::compute_FR_motor_speed(double P1, double s, double K_constant, double R, double multiplier)
{
    double FR_speed = ((P1 / s) * (1 - abs(R)) - R * K_constant) * -1;
    FR->move_voltage(FR_speed * 12700 * multiplier);
}

void DriveTrain::compute_BL_motor_speed(double P1, double s, double K_constant, double R, double multiplier)
{
    double BL_speed = ((P1 / s) * (1 - abs(R)) + R * K_constant);
    BL->move_voltage(BL_speed * 12700 * multiplier);
}

void DriveTrain::compute_BR_motor_speed(double P2, double s, double K_constant, double R, double multiplier)
{
    double BR_speed = ((P2 / s) * (1 - abs(R)) - R * K_constant) * -1;
    BR->move_voltage(BR_speed * 127000 * multiplier);
}

void DriveTrain::set_turn(int turn)
{
    FL->move_voltage(turn * 1000);
    FR->move_voltage(turn * 1000);
    BR->move_voltage(turn * 1000);
    BL->move_voltage(turn * 1000);
}

double DriveTrain::compute_angle_error(double target, double current_angle)
{
    double error = current_angle - target;
    if (error < -pi)
    {
        return -(error + TAU);
    }
    else if (error >= -pi && error <= 0)
    {
        return abs(error);
    }
    else if (error > 0 && error < pi)
    {
        return -error;
    }
    else
    {
        return abs(error - TAU);
    }
}

// function to drive to a point with and without turning:

void DriveTrain::drive_to_point(double tX, double tY, double target_angle_in_degrees, bool use_precise_turn, bool is_waypoint, const std::function<void()> &trigger, int trigger_distance, double timeout)
{
    //hyperparameters:
    const double rotational_KP = 2;
    const double translational_KP = 2;
    const double motors_on_off = 1;
    const double K_constant = 1;
    const double multiplier = 2;

    double prev_time = pros::millis();

    // when to slow down using a threshold:
    double slow_down_distance_threshold = 18;

    // acceptable distance error:
    double acceptable_distance_error = 0.2;

    if (is_waypoint == true)
    {
        acceptable_distance_error = 1.5;
        slow_down_distance_threshold = 3;
    }
    // setting the initial distance error:
    double initial_distance_error = sqrt(pow(tX - globalX, 2) + pow(tY - globalY, 2));

    // changing the target angle to radians:
    double target_angle = convert_deg_to_rad(target_angle_in_degrees);

    // declaring all of the error variables:
    double prev_angle_error, prev_distance_error, current_distance_error, angle_error, accumulated_error;

    //flag to see if the trigger function has activated
    bool trigger_activated = false;

    do
    {
        // variables that calculate the error in the X coordinate, Y coordinate, and angle:
        double error_in_X = tX - globalX;
        double error_in_Y = tY - globalY;
        prev_distance_error = current_distance_error;
        current_distance_error = sqrt(pow(error_in_X, 2) + pow(error_in_Y, 2));

        // rotational error (have arc_length_error so that we can convert angle error into inch):
        angle_error = compute_angle_error(target_angle, get_constrained_alpha());
        double arc_length_error = angle_error * wR;
        if (abs(current_distance_error) < 2)
        {
            arc_length_error = 0;
        }

        //ratio between the rotational and translational errors (tells how much motor power to apply to each):
        double R = MIN((arc_length_error * rotational_KP) / (15 + abs(current_distance_error)), 1); // formula was previously: 0.5 * arc_length_error / (current_distance_error + arc_length_error * 0.5);
        R = constrain(R, -1.0, 1.0);

        //how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational):
        double S;

        if (abs(current_distance_error) < slow_down_distance_threshold)
        {
            if (abs(current_distance_error) < 3)
            {
                accumulated_error += current_distance_error;
            }
            else
            {
                accumulated_error = 0;
            }

            S = ((abs(current_distance_error)) / (slow_down_distance_threshold * 1.7)) + abs(accumulated_error * 0.0015);

            S = constrain(S, 0.0, 1.0);
        }
        else
        {
            S = 1;
        }

        //Target coordinate plane offset:
        double T = atan2(error_in_Y, error_in_X) + alpha;

        run_Xdrive(T, S, R);

        //debugging:
        // pros::lcd::set_text(3, "distance error: " + std::to_string(current_distance_error));
        // pros::lcd::set_text(4, "R: " + std::to_string(R));
        // pros::lcd::set_text(5, "the error angle: " + std::to_string(convert_rad_to_deg(angle_error)));
        // pros::lcd::set_text(6, "alpha: " + std::to_string(convert_rad_to_deg(alpha)));
        // pros::lcd::set_text(7, "coordinates: (" + std::to_string(globalX) + ", " + std::to_string(globalY) + ")");

        //setting the previous values of the translational and rotational errors:
        prev_angle_error = angle_error;

        //conditional statement if you want to dispence balls at a certain distance away from a coordinate:
        if (trigger != 0 && current_distance_error < trigger_distance && !trigger_activated)
        {
            trigger();
            trigger_activated = true;
        }

        //delay (can be very small):
        pros::delay(20);
    } while ((abs(current_distance_error) > acceptable_distance_error) && abs(prev_time - pros::millis()) < timeout);

    //if you want to have much less final error in the target angle:
    if (use_precise_turn)
    {
        point_turn_PID(target_angle, 37.5, .7, 0);
    }

    //if you want to see when the function has exited:
    pros::lcd::set_text(0, "drive_to_point exited");
}

void DriveTrain::drive_to_tower_backboard(double target_angle)
{
    //turn to a specified angle
    // drivetrain.point_turn_PID(target_angle, true);

    //move laterally to center of the backboard:
    pros::vision_object_s_t backboard, prev_backboard;
    PID_controller pid_controller(0.01, 0, 0, 1, 0);
    do
    {
        backboard = vision_sensor.get_by_size(0);
        pros::lcd::set_text(7, "signature: " + std::to_string(backboard.signature));

        // if (backboard.signature == 3)
        // {
        //     prev_backboard = backboard;
        // }
        // else
        // {
        //     pros::lcd::set_text(7, "did not detect");
        //     backboard = prev_backboard;
        // }
        if (backboard.signature != 3)
        {
            pros::delay(20);
            FL->move_voltage(0 * 1000);
            FR->move_voltage(0 * 1000);
            BR->move_voltage(0 * 1000);
            BL->move_voltage(0 * 1000);
            continue;
        }

        pros::lcd::set_text(5, "area: " + std::to_string(backboard.width * backboard.height));
        pros::lcd::set_text(6, "vision coordinates: ( " + std::to_string(backboard.x_middle_coord) + ", " + std::to_string(backboard.y_middle_coord) + ")");

        double distance_error = backboard.x_middle_coord - 255;

        double angle_error = 0; // compute_angle_error(target_angle, convert_deg_to_rad(IMU.get_heading()));

        double T = atan2(0, distance_error);

        double S = pid_controller.compute(abs(distance_error));

        pros::lcd::set_text(3, "S: " + std::to_string(S));

        double arc_length_error = angle_error * wR;

        double R = MIN((arc_length_error * 1) / (15 + abs(distance_error)), 1);
        R = constrain(R, -1.0, 1.0);

        run_Xdrive(T, S, R);

        pros::delay(20);
    } while (true); //abs(pid_controller.get_error()) > 0.2);
}

void DriveTrain::run_Xdrive(double T, double S, double R)
{
    double P1 = compute_P1(T);
    double P2 = compute_P2(T);
    double s = compute_s(P1, P2, S);

    //how much power to apply to each motor:
    compute_FL_motor_speed(P2, s, 1, R, 2);
    compute_FR_motor_speed(P1, s, 1, R, 2);
    compute_BL_motor_speed(P1, s, 1, R, 2);
    compute_BR_motor_speed(P2, s, 1, R, 2);
}

void DriveTrain::set_translational_backboard_speed(double translational_speed)
{
    if (translational_speed < 0)
    {
        FL->move_voltage(-translational_speed * 1000);
        FR->move_voltage(-translational_speed * 1000);
        BR->move_voltage(translational_speed * 1000);
        BL->move_voltage(translational_speed * 1000);
    }
    if (translational_speed > 0)
    {
        FL->move_voltage(translational_speed * 1000);
        FR->move_voltage(translational_speed * 1000);
        BR->move_voltage(-translational_speed * 1000);
        BL->move_voltage(-translational_speed * 1000);
    }
}

// void DriveTrain::drive_to_tower_backboard(double IMU_angle_to_turn)
// {
//     double error_in_coordinates, difference_in_coordinates, accumulated_error, previous_error_in_coordinates = 0;
//     pros::vision_object_s_t backboard;
//     do
//     {
//         backboard = vision_sensor.get_by_size(0);
//         double X = backboard.x_middle_coord;
//         drivetrain.point_turn_PID(IMU_angle_to_turn, 40, .7, -43, true);

//         error_in_coordinates = 255 - X;
//         difference_in_coordinates = previous_error_in_coordinates - error_in_coordinates;
//         if (abs(error_in_coordinates) < 20)
//         {
//             accumulated_error += error_in_coordinates;
//         }
//         else
//         {
//             accumulated_error = 0;
//         }

//         if ((previous_error_in_coordinates < 0 && error_in_coordinates > 0) || (error_in_coordinates < 0 && previous_error_in_coordinates > 0))
//             accumulated_error = 0;

//         set_translational_backboard_speed(error_in_coordinates * 1 + accumulated_error * 0.001 + difference_in_coordinates);

//         previous_error_in_coordinates = error_in_coordinates;
//     } while ((abs(difference_in_coordinates) > 0.0001) || abs(error_in_coordinates) > 0.01);
// }

void set_translational_backboard_speed(double speed_to_translate);

// void DriveTrain::point_turn_PID(double target, const double Kp, const double Ki, const double Kd, bool use_IMU)
// {
//     double error, derivative, integral, change_time, previous_time, previous_error;
//     do
//     {
//         if (use_IMU)
//         {
//             error = compute_angle_error(target, convert_deg_to_rad(IMU.get_heading()));
//         }
//         else
//         {
//             error = compute_angle_error(target, get_constrained_alpha());
//         }

//         derivative = (previous_error - error);

//         if (abs(error) < 0.349066) // 20 degrees
//         {
//             integral += error;
//         }
//         else
//         {
//             integral = 0;
//         }

//         if ((previous_error < 0 && error > 0) || (error < 0 && previous_error > 0))
//             integral = 0;

//         set_turn(error * Kp + integral * Ki + derivative * Kd);

//         // pros::lcd::set_text(6, "the error is: " + std::to_string(error * 180 / pi));
//         // pros::lcd::set_text(7, "the target is: " + std::to_string(convert_rad_to_deg_wraped(target)));
//         // pros::lcd::set_text(4, "the derivative is: " + std::to_string(derivative));
//         // pros::lcd::set_text(3, "the integral is: " + std::to_string(accumulated_error));
//         // pros::lcd::set_text(5, "PID value is: " + std::to_string(error * Kp + integral * Ki + derivative * Kd));
//         // pros::lcd::set_text(2, "alpha is: " + std::to_string(get_alpha_in_degrees()));

//         previous_error = error;

//         pros::delay(20);

//     } while (((abs(derivative) > 0.0001) || abs(error) > 0.01));

//     pros::lcd::set_text(0, "pid escaped");
// }

void DriveTrain::point_turn_PID(double target, bool use_IMU, const double Kp, const double Ki, const double Kd)
{
    PID_controller pid_controller(Kp, Ki, Kd, 127, -127);
    pid_controller.use_integrater_error_bound(convert_deg_to_rad(5));
    pid_controller.use_crossover_zero();
    double angle_error;
    do
    {
        if (use_IMU)
        {
            angle_error = compute_angle_error(convert_deg_to_rad(target), convert_deg_to_rad(IMU.get_heading()));
        }
        else
        {
            angle_error = compute_angle_error(convert_deg_to_rad(target), get_constrained_alpha());
        }
        angle_error = compute_angle_error(convert_deg_to_rad(target), get_constrained_alpha());
        double motor_power = pid_controller.compute(angle_error, false);

        set_turn(motor_power);
        pros::lcd::set_text(4, "the angle error is: " + std::to_string(angle_error * 180 / pi));
        pros::lcd::set_text(3, "the PID value is: " + std::to_string(motor_power));
        pros::delay(20);
    } while (abs(pid_controller.get_derivative()) > 0.000000000001 || abs(angle_error) > 0.004);
    pros::lcd::set_text(0, "pid escaped");
}

void DriveTrain::start_odometry_update_thread()
{
    while (true)
    {
        update_odometry();
        pros::Task::delay(10);
    }
}

void DriveTrain::make_odometry_update_thread()
{
    task = std::make_shared<pros::Task>([this] { start_odometry_update_thread(); });
}

//getter for globalX:

double DriveTrain::get_globalX()
{
    return globalX;
}

double DriveTrain::get_constrained_alpha()
{
    double theta = alpha;
    theta = fmod(theta, TAU);
    if (theta < 0)
    {
        theta += TAU;
    }
    return theta;
}

//getter for globalY

double DriveTrain::get_globalY()
{
    return globalY;
}

//destructor:

DriveTrain::~DriveTrain()
{
    if (task.get() != nullptr)
    {
        task->remove();
    }
}
