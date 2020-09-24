#include <iostream>
#include "main.h"
#include "auton.h"
#include "globals.h"
#include "PID_controller.h"

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

DriveTrain::DriveTrain(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor *FL, pros::Motor *FR, pros::Motor *BL, pros::Motor *BR, pros::ADIEncoder *encoderL, pros::ADIEncoder *encoderR, pros::ADIEncoder *encoderM, pros::Vision *vision_sensor, pros::Imu *IMU, pros::ADIAnalogIn *left_pot, pros::ADIAnalogIn *right_pot, pros::ADIAnalogIn *collision_light_sensor)
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
    this->vision_sensor = vision_sensor;
    this->IMU = IMU;
    this->left_pot = left_pot;
    this->right_pot = right_pot;
    this->collision_light_sensor = collision_light_sensor;
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
    // return convert_rad_to_deg_wraped(alpha);
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
    delta_left_encoder_distance = left_encoder_distance - prev_left_encoder_distance;

    double right_encoder_distance = get_right_encoder_distance();
    delta_right_encoder_distance = right_encoder_distance - prev_right_encoder_distance;

    double middle_encoder_distance = get_middle_encoder_distance();
    delta_middle_encoder_distance = middle_encoder_distance - prev_middle_encoder_distance;

    double delta_alpha, delta_IMU_heading;

    delta_IMU_heading = compute_angle_error(convert_deg_to_rad(IMU->get_heading()), prev_IMU_heading);
    IMU_heading += delta_IMU_heading;

    if (is_IMU_odometry)
    {
        delta_alpha = delta_IMU_heading;
    }
    else
    {
        delta_alpha = compute_delta_alpha(delta_right_encoder_distance, delta_left_encoder_distance);
    }

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
    prev_IMU_heading = IMU_heading;
    update_odometry_mutex.give();
}

void DriveTrain::use_IMU_for_odometry(bool is_IMU_odometry)
{
    this->is_IMU_odometry = is_IMU_odometry;
}

void DriveTrain::set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees)
{
    update_odometry_mutex.take(1000);
    globalX = new_X;
    globalY = new_Y;
    prev_alpha = convert_deg_to_rad(new_alpha_in_degrees);
    alpha = prev_alpha;
    prev_IMU_heading = convert_deg_to_rad(new_alpha_in_degrees);
    IMU_heading = prev_IMU_heading;
    update_odometry_mutex.give();
}

void DriveTrain::set_alpha(double new_alpha_in_degrees)
{
    update_odometry_mutex.take(1000);
    prev_IMU_heading = convert_deg_to_rad(new_alpha_in_degrees);
    IMU_heading = prev_IMU_heading;
    prev_alpha = convert_deg_to_rad(new_alpha_in_degrees);
    alpha = prev_alpha;
    update_odometry_mutex.give();
}

void DriveTrain::reset_odom()
{
    set_current_global_position(0, 0, 0);
}

void DriveTrain::turn_to_point(double destX, double destY)
{
    double angle = atan2(destX - globalX, destY - globalY);
    if (angle < 0)
    {
        angle += TAU;
    }
    pros::lcd::set_text(5, "the angle to turn is: " + std::to_string(convert_rad_to_deg_wraped(angle)));
    point_turn_PID(convert_rad_to_deg_wraped(angle));
}

/*
functions to calculate intermediate steps:

P1 = -cos(T + pi/4)

P2 = sin(T + pi/4)

s = max(std::abs(P1), std::abs(P2)) / S
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
    return MAX(std::abs(P1), std::abs(P2)) / S;
}

/*
    functions to calculate the speed to turn the motors:

    FL = P2/s(1 - std::abs(R)) + R * S

    FR = P1/s(1 - std::abs(R)) - R * S

    BL = P1/s(1 - std::abs(R)) + R * S
    
    BR = P2/s(1 - std::abs(R)) - R * S
*/

void DriveTrain::compute_FL_motor_speed(double P2, double s, double K_constant, double R, double multiplier)
{
    double FL_speed = (P2 / s) * (1 - std::abs(R)) + R * K_constant;
    FL->move_voltage(FL_speed * 12700 * multiplier);
    // pros::lcd::set_text(0, "FL:" + std::to_string(FL_speed));
}

void DriveTrain::compute_FR_motor_speed(double P1, double s, double K_constant, double R, double multiplier)
{
    double FR_speed = ((P1 / s) * (1 - std::abs(R)) - R * K_constant) * -1;
    FR->move_voltage(FR_speed * 12700 * multiplier);
    // pros::lcd::set_text(1, "FR:" + std::to_string(FR_speed));
}

void DriveTrain::compute_BL_motor_speed(double P1, double s, double K_constant, double R, double multiplier)
{
    double BL_speed = ((P1 / s) * (1 - std::abs(R)) + R * K_constant);
    BL->move_voltage(BL_speed * 12700 * multiplier);
    // pros::lcd::set_text(2, "BL:" + std::to_string(BL_speed));
}

void DriveTrain::compute_BR_motor_speed(double P2, double s, double K_constant, double R, double multiplier)
{
    double BR_speed = ((P2 / s) * (1 - std::abs(R)) - R * K_constant) * -1;
    BR->move_voltage(BR_speed * 12700 * multiplier);
    // pros::lcd::set_text(3, "BR:" + std::to_string(BR_speed));
}

void DriveTrain::set_turn(double turn)
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
        return std::abs(error);
    }
    else if (error > 0 && error < pi)
    {
        return -error;
    }
    else
    {
        return std::abs(error - TAU);
    }
}

void DriveTrain::run_Xdrive(double T, double S, double R, double K_constant)
{
    double P1 = compute_P1(T);
    double P2 = compute_P2(T);
    double s = compute_s(P1, P2, S);

    // pros::lcd::set_text(0, "s:" + std::to_string(s));
    // pros::lcd::set_text(1, "P2: " + std::to_string(P2));
    // pros::lcd::set_text(2, "P1: " + std::to_string(P1));
    // pros::lcd::set_text(3, "R: " + std::to_string(R));
    // pros::lcd::set_text(5, "MAX(std::abs(P1), std::abs(P2)): " + std::to_string(MAX(std::abs(P1), std::abs(P2))));

    //how much power to apply to each motor:
    compute_FL_motor_speed(P2, s, K_constant, R, 2);
    compute_FR_motor_speed(P1, s, K_constant, R, 2);
    compute_BL_motor_speed(P1, s, K_constant, R, 2);
    compute_BR_motor_speed(P2, s, K_constant, R, 2);
}

void DriveTrain::drive_to_point(double tX, double tY, double target_angle_in_degrees, int point_type, double rotational_KP, const std::function<void()> &trigger, double trigger_distance, double timeout)
{
    //configuration:
    //hyperparameters:
    const double translational_KP = 2;
    const double motors_on_off = 1;
    const double K_constant = 1;
    const double multiplier = 2;
    const double acceptable_angle_error = 1;
    const double speed_up_time = 250;
    const bool use_motion_profiling = false;

    double slow_down_distance_threshold = 18;
    double acceptable_distance_error = 0.2;

    //waypoint point type:
    if (point_type == 1)
    {
        acceptable_distance_error = 1.5;
        slow_down_distance_threshold = 3;
    }

    //in between waypoint and destination point:
    else if (point_type == 2)
    {
        acceptable_distance_error = 0.2;
        slow_down_distance_threshold = 10;
    }

    //PID controllers:
    PID_controller S_controller(1 / (slow_down_distance_threshold * 1.6), 0.0015, 0, 1, 0.1);

    //constraining integral for S_controller:
    S_controller.use_integrater_error_bound(3);
    S_controller.use_crossover_zero();

    // utilities:
    double start_time = pros::millis();
    double initial_distance_error = sqrt(pow(tX - globalX, 2) + pow(tY - globalY, 2));
    double target_angle = convert_deg_to_rad(target_angle_in_degrees);
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
        // if (std::abs(current_distance_error) < 2)
        // {
        //     arc_length_error = 0;
        // }

        //ratio between the rotational and translational errors (tells how much motor power to apply to each):
        double R = (arc_length_error * rotational_KP) / (15 + std::abs(current_distance_error));

        R = constrain(R, -1.0, 1.0);

        //how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational):
        double S;

        if (std::abs(current_distance_error) < slow_down_distance_threshold)
        {
            S = S_controller.compute(std::abs(current_distance_error));
        }
        else
        {
            S = 1;
        }

        //motion profiling
        if (use_motion_profiling)
        {
            double millis = pros::millis() - start_time;
            if (speed_up_time != 0 && millis < speed_up_time && abs(current_distance_error) > slow_down_distance_threshold)
            {
                S = millis / speed_up_time;
                // if (use_limited_acceleration)
                // {
                //     // K_constant = S;
                // }
            }
        }

        //Target coordinate plane offset:
        double T = atan2(error_in_Y, error_in_X) + alpha;

        run_Xdrive(T, S, R, K_constant);

        //debugging:
        // pros::lcd::set_text(3, "distance error: " + std::to_string(current_distance_error));
        // pros::lcd::set_text(4, "R: " + std::to_string(R));
        // pros::lcd::set_text(5, "the error angle: " + std::to_string(convert_rad_to_deg(angle_error)));
        // pros::lcd::set_text(6, "alpha: " + std::to_string(convert_rad_to_deg(alpha)));
        // pros::lcd::set_text(7, "coordinates: (" + std::to_string(globalX) + ", " + std::to_string(globalY) + ")");

        prev_angle_error = angle_error;

        //conditional statement if you want to trigger custom function at a certain distance away from a coordinate:
        if (trigger != 0 && current_distance_error < trigger_distance && !trigger_activated)
        {
            trigger();
            trigger_activated = true;
        }

        pros::delay(20);
    } while (((std::abs(current_distance_error) > acceptable_distance_error)) && std::abs(start_time - pros::millis()) < timeout);

    pros::lcd::set_text(0, "drive_to_point exited");
}

bool DriveTrain::collision_detected()
{
    double light_sensor_threshold;
    return (delta_left_encoder_distance == 0 && delta_right_encoder_distance == 0 && delta_middle_encoder_distance == 0);
}

bool DriveTrain::is_L_pot_bending()
{
    double L_pot_threshold = 700;
    double L_pot_error = L_pot_threshold - left_pot->get_value();
    return L_pot_error < -30;
}

bool DriveTrain::is_R_pot_bending()
{
    double R_pot_threshold = 750;
    double R_pot_error = R_pot_threshold - right_pot->get_value();
    return R_pot_error < -30;
}

void DriveTrain::center_on_tower_with_bumper(double target_angle, bool use_IMU, double timeout, bool use_pots)
{
    double L_pot_threshold = 700;
    double R_pot_threshold = 750;
    double prev_time = pros::millis();
    PID_controller pot_L_controller(0.005, 0, 0, 1, 0);
    PID_controller pot_R_controller(0.005, 0, 0, 1, 0);
    do
    {
        double L_pot_error = L_pot_threshold - left_pot->get_value();
        double R_pot_error = R_pot_threshold - right_pot->get_value();

        bool L_pot_bend_detected = use_pots? is_L_pot_bending() : false;
        bool R_pot_bend_detected = use_pots? is_R_pot_bending() : false;

        double angle_error = compute_angle_error(convert_deg_to_rad(target_angle), use_IMU ? convert_deg_to_rad(IMU->get_heading()) : get_constrained_alpha());
        double arc_length_error = angle_error * wR;

        //ratio between the rotational and translational errors (tells how much motor power to apply to each):
        double R;
        double S;
        double T;

        double L_bend_time, R_bend_time;

        if (L_pot_bend_detected)
        {
            L_bend_time = 200 + pros::millis();
        }
        if (R_pot_bend_detected)
        {
            R_bend_time = 200 + pros::millis();
        }
        

        if (L_bend_time > pros::millis())
        {
            R = MIN((arc_length_error * 1) / (std::abs(L_pot_error * 0.01)), 1);
            S = pot_L_controller.compute(std::abs(L_pot_error));
            T = atan2(-10, 1);
        }
        else if (R_bend_time > pros::millis())
        {
            R = MIN((arc_length_error * 1) / (std::abs(R_pot_error * 0.01)), 1);
            S = pot_R_controller.compute(std::abs(R_pot_error));
            T = atan2(-10, -1);
        }

        if (L_pot_bend_detected && R_pot_bend_detected)
        {
            R = MIN((arc_length_error * 1) / 30, 1);
            S = 0.3;
            T = atan2(-10, ((std::abs(R_pot_error) / -std::abs(R_pot_error)) / 2) * -0.001);
        }

        if (!R_pot_bend_detected && !(L_bend_time > pros::millis()))
        {
            R = MIN((arc_length_error * 1) / 1000, 1);
            S = 0.3;
            T = atan2(10, 0);
        }

        R = constrain(R, -1.0, 1.0);
        S = constrain(S, 0.0, 1.0);
        // pros::lcd::set_text(0, "T:" + std::to_string(T));
        pros::lcd::set_text(3, "R: " + std::to_string(R));
        // pros::lcd::set_text(1, "S: " + std::to_string(S));

        run_Xdrive(T, S, R);
        pros::delay(20);

    } while (std::abs(prev_time - pros::millis()) < timeout); //(!collision_detected() || is_R_pot_bending() || is_L_pot_bending() || std::abs(pros::millis() - prev_time) < 150);
    stop_drive_motors();
    pros::lcd::set_text(3, "exited");
}

void DriveTrain::drive_to_tower_backboard(double target_angle, double when_to_include_integral, bool use_IMU, bool is_bumper_drive)
{
    //turn to a specified angle
    point_turn_PID(target_angle, use_IMU);

    //set acceptable_error
    double acceptable_Xerror, acceptable_average_Xerror, acceptable_S, X_center_position, S_increaser;
    if (is_bumper_drive)
    {
        acceptable_Xerror = 2;
        acceptable_average_Xerror = 2;
        acceptable_S = 0.5;
        X_center_position = 214;
        S_increaser = 0.2;
    }
    else
    {
        acceptable_Xerror = 0.6;
        acceptable_average_Xerror = 0.6;
        acceptable_S = 0.15;
        X_center_position = 206;
        S_increaser = 0;
    }

    pros::vision_object_s_t backboard = vision_sensor->get_by_size(0);

    //driving forward to the center of the game tower:
    double prev_time = pros::millis();
    double X_error;

    PID_controller pid_controller(0.012, 0.0002, 0, 1, 0); //0.000097

    pid_controller.use_crossover_zero();
    pid_controller.use_integrater_error_bound(when_to_include_integral); //first time == 3, second time == 7, third time == 7
    double S;
    SimpleKalmanFilter vision_kalman_filter(3, 3, 0.08);
    do
    {
        backboard = vision_sensor->get_by_size(0);
        if (backboard.signature != 3)
        {
            pros::delay(20);
            X_error = 50;
        }
        else
        {
            double filtered_X = vision_kalman_filter.updateEstimate(backboard.x_middle_coord);
            pros::lcd::set_text(5, "average error: " + std::to_string(pid_controller.get_error_average(10)));
            pros::lcd::set_text(2, "X:" + std::to_string(backboard.x_middle_coord) + "f: " + std::to_string(filtered_X) + ")");
            X_error = filtered_X - X_center_position;
        }

        // double angle_error = compute_angle_error(convert_deg_to_rad(target_angle), convert_deg_to_rad(IMU->get_heading()))
        double angle_error = compute_angle_error(convert_deg_to_rad(target_angle), use_IMU ? convert_deg_to_rad(IMU->get_heading()) : get_constrained_alpha());

        double arc_length_error = angle_error * wR;

        double T = atan2(.5, X_error);
        S = pid_controller.compute(std::abs(X_error));
        double R = MIN((arc_length_error * 2) / (15 + std::abs(X_error)), 1);

        S = constrain(S, 0.0, 1.0);

        pros::lcd::set_text(3, "S: " + std::to_string(S));
        pros::lcd::set_text(4, std::to_string((int)FL->get_voltage()) + " " + std::to_string((int)FR->get_voltage()) + " " + std::to_string((int)BL->get_voltage()) + " " + std::to_string((int)BR->get_voltage()));
        R = constrain(R, -1.0, 1.0);

        run_Xdrive(T, S, R);
        pros::delay(20);
        printf("%d,%f,%d\n", pros::millis(), X_error, 0);
    } while (!((std::abs(X_error) < acceptable_Xerror) && S < acceptable_S && pid_controller.get_error_average(10) < acceptable_average_Xerror));

    pros::lcd::set_text(7, "exited");

    if (is_bumper_drive)
    {
        set_motors(15, -15, 15, -15);
        pros::delay(100);
        // point_turn_PID(target_angle, use_IMU);
    }
    stop_drive_motors();
}

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
            angle_error = compute_angle_error(convert_deg_to_rad(target), convert_deg_to_rad(IMU->get_heading()));
        }
        else
        {
            angle_error = compute_angle_error(convert_deg_to_rad(target), get_constrained_alpha());
        }
        angle_error = compute_angle_error(convert_deg_to_rad(target), get_constrained_alpha());
        double motor_power = pid_controller.compute(angle_error);

        set_turn(motor_power);
        pros::lcd::set_text(4, "the angle error is: " + std::to_string(angle_error * 180 / pi));
        pros::lcd::set_text(3, "the PID value is: " + std::to_string(motor_power));
        pros::delay(20);
    } while (std::abs(pid_controller.get_derivative()) > 0.000000000001 || std::abs(angle_error) > 0.004);
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
    odometry_update_task = std::make_shared<pros::Task>([this] { start_odometry_update_thread(); });
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

void DriveTrain::set_motors(double FL_motor_power, double FR_motor_power, double BL_motor_power, double BR_motor_power)
{
    FL->move_voltage(FL_motor_power * 1000);
    FR->move_voltage(FR_motor_power * 1000);
    BL->move_voltage(BL_motor_power * 1000);
    BR->move_voltage(BR_motor_power * 1000);
}
void DriveTrain::stop_drive_motors()
{
    set_motors(0, 0, 0, 0);
}

void DriveTrain::driver_control(double Yaxis, double Xaxis, double turn)
{

    double FL_power = Yaxis + Xaxis + (turn);
    double FR_power = -Yaxis + Xaxis + (turn);
    double BL_power = Yaxis - Xaxis + (turn);
    double BR_power = -Yaxis - Xaxis + (turn);

    FL->move(FL_power);
    FR->move(FR_power);
    BL->move(BL_power);
    BR->move(BR_power);
}

void DriveTrain::calibrate_IMU()
{
    pros::lcd::set_text(5, "Calibrating IMU");
    IMU->reset();

    while (IMU->is_calibrating())
    {
        pros::delay(10);
    }
    IMU->set_data_rate(5);
}

void filter_IMU()
{
}

double DriveTrain::get_IMU_heading()
{
    return convert_rad_to_deg_wraped(IMU_heading);
}

void DriveTrain::setup_sensors()
{

    BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
    RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);
    tower_backboard_signature = pros::Vision::signature_from_utility(3, -4013, -3667, -3840, -5007, -4469, -4738, 11.000, 0);
    vision_sensor->set_exposure(60);
    vision_sensor->set_signature(3, &tower_backboard_signature);
}

void DriveTrain::setup()
{
    setup_sensors();
    make_odometry_update_thread();
}

//destructor:
DriveTrain::~DriveTrain()
{
    if (odometry_update_task.get() != nullptr)
    {
        odometry_update_task->remove();
    }
}
