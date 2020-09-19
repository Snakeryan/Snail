#ifndef drivetrain_H
#define drivetrain_H

/******************************************************************************/
/**           General methods for smooth, autonomous movements               **/
/**                                                                          **/
/**       These methods allow for absolute tracking and positioning          **/
/******************************************************************************/
#include <pthread.h>
#include <memory>
#include "main.h"
#include "SimpleKalmanFilter.h"

class DriveTrain
{

    // Task that updates the odometry functions:
    pros::Mutex update_odometry_mutex;

    double encoder_wheel_radius, wL, wR, wM, globalX, globalY, alpha, prev_alpha, IMU_heading, prev_IMU_heading, prev_left_encoder_distance, delta_left_encoder_distance, prev_right_encoder_distance, delta_right_encoder_distance, prev_middle_encoder_distance, delta_middle_encoder_distance = 0;

    // SimpleKalmanFilter heading_filter = simplekalmanfilter(1, 1, 0.01);

    // flag for running odometry with and without the IMU:
    bool is_IMU_odometry;

    //shared pointer to ensure that the update_odometry task(s) will never outlive the function
    std::shared_ptr<pros::Task>
        odometry_update_task{nullptr};

    // motor/sensor parameters:
    pros::Motor *FL;
    pros::Motor *FR;
    pros::Motor *BL;
    pros::Motor *BR;
    pros::ADIEncoder *encoderL;
    pros::ADIEncoder *encoderR;
    pros::ADIEncoder *encoderM;

    pros::Vision *vision_sensor;
    pros::vision_signature_s_t BLUE_BALL_SIGNATURE;
    pros::vision_signature_s_t RED_BALL_SIGNATURE;
    pros::vision_signature_s_t tower_backboard_signature;

    pros::Imu *IMU;

    pros::ADIAnalogIn *collision_light_sensor;
    pros::ADIAnalogIn *left_pot;
    pros::ADIAnalogIn *right_pot;

    void run_Xdrive(double T, double S, double R, double K_constant = 1);
    /**
 *        this method takes inches as its units and it uses this formula: (delta_left_distance - delta_right_distance) / (wL + wR)
 * \param delta_right_distance
 *        the change in the value of the distance of the right encoder (takes inches)
 * \param delta_left_distance
 *        the change in the value of the distance of the left encoder (takes inches)
 *
 * \return 
 *        the change in the robot's heading
*/
    double compute_delta_alpha(double delta_right_distance, double delta_left_distance);

    /**
 *        this is the formula used by this method: (2 * sin(delta_alpha / 2)) * (wR + (delta_right_distance /delta_alpha))
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \param delta_right_distance
 *        the change in the value of the distance of the right encoder (takes inches)
 * \return 
 *        the change in the robot's local Y coordinate/position 
*/
    double compute_delta_Dly(double delta_alpha, double delta_right_distance);

    /**
 *        this is the formula used by this method: (2 * sin(delta_alpha / 2)) * ((delta_middle_distance / delta_alpha) + wM)
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \param delta_middle_distance
 *        the change in the value of the distance of the right encoder (takes inches)
 * \return 
 *        the change in the local X coordinate/position
*/
    double compute_delta_Dlx(double delta_alpha, double delta_middle_distance);

    /**
 *        this is the formula used by this method: (Dlx * cos(delta_alpha/2 + prev_alpha)) + (Dly * sin(delta_alpha/2 + prev_alpha))
 * \param Dlx
 *        the change in the local X coordinate/position
 * \param Dly
 *        the change in the local Y coordinate/position
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \return 
 *        the change in the absolute, or global, X coordinate/position
*/
    double compute_delta_globalX(double Dlx, double Dly, double delta_alpha);

    /**
 *        this is the formula used by this method: (Dly * cos(delta_alpha/2 + prev_alpha)) - (Dlx * sin(delta_alpha/2 + prev_alpha))
 * \param Dlx
 *        the change in the local X coordinate/position
 * \param Dly
 *        the change in the local Y coordinate/position
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \return 
 *        the change in the absolute, or global, Y coordinate/position
*/
    double compute_delta_globalY(double Dlx, double Dly, double delta_alpha);

    /**
 *        this is the formula used by this method: -cos(T + pi/4)
 * \param T
 *        How much the current coordinate plane needs to be offset by to fix the robot's directional perspective
 * \return 
 *        value calculated for smooth movements of the robot's front right and back left wheels
*/
    double compute_P1(double T);

    /**
 *        this is the formula used by this method: sin(T + pi/4)
 * \param T
 *        How much the current coordinate plane needs to be offset by to fix the robot's directional perspective
 * \return 
 *        value calculated for smooth movements of the robot's front left and back right wheels
*/
    double compute_P2(double T);

    /** 
 *        this is the formula used by this method: MAX(abs(P1), abs(P2)) / S
 * \param P1
 *        value calculated for smooth movements of the robot's front right and back left wheels
 * \param P2
 *        value calculated for smooth movements of the robot's front left and back right wheels
 * \param S
 *        how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \return 
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
*/
    double compute_s(double P1, double P2, double S);

    /** 
 *        this is the formula used by this method: (P2 / s) * (1 - abs(R)) + R * K_constant
 * \param P2
 *        value calculated for smooth movements of the robot's front left and back right wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to tell how much to increase the overall voltage that goes to the motors
 * \param use_motor
 *       a debugging parameter that turns the motors off if it is zero and on if it is one (one is the default parameter)
*/
    void compute_FL_motor_speed(double P2, double s, double K_constant, double R, double multiplier);

    /** 
 *        this is the formula used by this method: ((P1 / s) * (1 - abs(R)) - R * K_constant) * -1
 * \param P1
 *        value calculated for smooth movements of the robot's front right and back left wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to tell how much to increase the overall voltage that goes to the motors
 * \param use_motor
 *       a debugging parameter that turns the motors off if it is zero and on if it is one (one is the default parameter)
*/
    void compute_FR_motor_speed(double P1, double s, double K_constant, double R, double multiplier);

    /** 
 *        this is the formula used by this method: ((P1 / s) * (1 - abs(R)) + R * K_constant)
 * \param P1
 *        value calculated for smooth movements of the robot's front right and back left wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to tell how much to increase the overall voltage that goes to the motors
 * \param use_motor
 *       a debugging parameter that turns the motors off if it is zero and on if it is one (one is the default parameter)
*/
    void compute_BL_motor_speed(double P1, double s, double K_constant, double R, double multiplier);

    /** 
 *        this is the formula used by this method: ((P2 / s) * (1 - abs(R)) - R * K_constant) * -1
 * \param P2
 *        value calculated for smooth movements of the robot's front left and back right wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to tell how much to increase the overall voltage that goes to the motors
 * \param use_motor
 *       a debugging parameter that turns the motors off if it is zero and on if it is one (one is the default parameter)
*/
    void compute_BR_motor_speed(double P2, double s, double K_constant, double R, double multiplier);

    /** 
 *        main funcition for odometry, as it is where all of the variables are updated and absolute coordinates are calculated
*/
    void update_odometry();

    /**
 *        this is the formula used by this method: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert to degrees
 * \return 
 *        the degree version of the radian value (method also constrains this version between 0-360)
*/
    double convert_rad_to_deg_wraped(double rad);

    /**
 *        this is the formula used by this method: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert to degrees
 * \return 
 *        the degree version of the radian value 
*/
    double convert_rad_to_deg(double rad);

    /**
 *        this is the formula used by this method: (deg * pi)/180
 * 
 * \param deg
 *        the degree value that you want to convert to radians
 * \return 
 *        the radian version of the degree value (does not wrap the radians between 0-2π)
*/
    double convert_deg_to_rad(double deg);

    /**
 *        this is the formula used by this method: (left_encoder_distance - right_encoder_distance) / (wL + wR)
 * 
 * \param right_encoder_distance 
 *        the change in the value of the distance of the right encoder (takes inches)
 * \param left_encoder_distance
 *        the change in the value of the distance of the left encoder (takes inches)
 * \return 
 *        the absolute/non-delta version of the robot's heading
*/
    double compute_alpha(double right_encoder_distance, double left_encoder_distance);

    /**
 * \param turn
 *        the speed at which you want the robot to turn (12 is the maximum speed)
*/
    void set_turn(double turn);

    /**
     * always returns a value between 0-2π
 * \param turn
 *        the speed at which you want the robot to turn (12 is the maximum speed)
*/
    double compute_angle_error(double target, double current_angle);

    /**
 * \return 
 *        the radian version of the heading of the robot (this is constrained from 0-2π)
*/
    double get_constrained_alpha();

    void setup_sensors();
    /**
 *        this function calls start_odometry_update_thread() to create a seperate task for updating the odometry variables
*/
    void make_odometry_update_thread();

public:
    /** 
 *        this puts the update_odometry function into a while(true) loop with a very small delay of ten miliseconds 
*/
    void start_odometry_update_thread();
    /** 
 *        this method calls all of the motion algorithm functions and allows you to smoothly turn and drive to a point (requires X-drive)
 * \param tX
 *        X coordinate that you want the robot to move to
 * \param tY
 *        Y coordinate that you want the robot to move to
 * \param target_angle_in_degrees
 *        the angle you want the robot to turn to while moving
 * \param use_precise_turn
 *        adds a PID loop to the end of the drive_to_point function that allows for precise turns 
 * \param is_waypoint
 *        if the point you are traveling to is a waypoint to get to another point, set this parameter to true (will only move translationally)
 * \param trigger
 *        allows you to pass in a function that will happen at a certain distance away from the target point (can pass in a lambda and pass in NULL if you do not want to run a function)
 * \param trigger_distance
 *        the distance away from the target coordinate that you want to activate the function you passed in (put 0 if you do not want to pass in anything)
 * \param timeout
 *       the maximum amount of time the robot can spend driving to a point (has default of 10000 milliseconds`)
*/
    void drive_to_point(double tX, double tY, double target_angle_in_degrees, int point_type = 2, double rotational_KP = 1, bool use_limited_acceleration = true, const std::function<void()> &trigger = 0, double trigger_distance = 3, double timeout = 10000);

    /** 
 *        this method changes where the robot thinks it is with respect to its coordinates and heading (only use this if you are starting in a new position)
 * \param new_X
 *        new X coordinate that you want to set to the robot
 * \param new_Y
 *        new Y coordinate that you want to set to the robot
 * \param new_alpha_in_degrees
 *        new heading that you want to set to the robot
*/
    void set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees);

    /** 
 * \param encoder_wheel_radius
 *        the radius of the tracking wheels (takes inches and if you are using the smallest wheels, they have a radius of 1.375 inches)
 * \param wL
 *        the distance from the center of the robot to the center of the left tracking wheel 
 * \param wR
 *        the distance from the center of the robot to the center of the right tracking wheel
 * \param wM
 *        the distance from the center of the robot to the center of the middle/back tracking wheel
 * \param FL
 *        the address of the front left motor
 * \param FR
 *        the address of the front right motor
 * \param BL 
 *        the address of the back left motor
 * \param BR
 *        the address of the back right motor
 * \param encoderL
 *        the address of the left encoder
 * \param encoderR
 *        the address of the right encoder
 * \param encoderM
 *        the address of the middle encoder
*/
    DriveTrain(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor *FL, pros::Motor *FR, pros::Motor *BL, pros::Motor *BR, pros::ADIEncoder *encoderL, pros::ADIEncoder *encoderR, pros::ADIEncoder *encoderM, pros::Vision *vision_sensor, pros::Imu *IMU, pros::ADIAnalogIn *left_pot, pros::ADIAnalogIn *right_pot, pros::ADIAnalogIn *collision_light_sensor);

    /**  
 * \return the heading of the robot in degrees (wrapped from 0-360)
*/
    double get_alpha_in_degrees();

    /** 
 *       \return the global/absolute X coordinate of the robot
*/
    double get_globalX();

    /** 
 *       \return the global/absolute Y coordinate of the robot
*/
    double get_globalY();

    /** 
 *       \return the distance the left encoder has traveled in inches
*/
    double get_left_encoder_distance();

    /** 
 *       \return the distance the right encoder has traveled in inches
*/
    double get_right_encoder_distance();

    /** 
 *       \return the distance the middle encoder has traveled in inches
*/
    double get_middle_encoder_distance();

    void drive_to_tower_backboard(double IMU_angle_to_turn, double when_to_include_integral, bool use_IMU = false, bool is_bumper_drive = true);

    /** 
 *        this method is a PID loop for turning to a specific angle
 * \param target
 *        the target angle you want the robot to face (in degrees)
 * \param use_IMU
 *        if you want to not use the angle calculated by odometry and use the IMU, then set this parameter to true
 * \param Kp
 *        the Proportional control constant to multiply by (has default of 37.5, but recommend changing to work for your robot) 
 * \param Ki
 *        the integral control constant to multiply by (has default of 0.7, but recommend changing to work for your robot) 
 * \param Kd
 *        the derivative control constant to multiply by (has default of -0, as this constant is supposed to be negative)
*/
    void point_turn_PID(double target, bool use_IMU = false, const double Kp = 30, const double Ki = 1.1, const double Kd = -65);

    /** 
 *        this method uses a PID loop to turn to a point on the coordinate system
 * \param X2
 *        the X coordinate you want to turn to
 * \param Y2
 *        the Y coordinate you want to turn to
*/
    void turn_to_point(double X2, double Y2);

    /** 
 *        a destructor to make sure that the update_odometry task never outlives the object
*/
    void set_motors(double FL_motor_power, double FR_motor_power, double BL_motor_power, double BR_motor_power);

    bool collision_detected();

    bool is_L_pot_bending();

    bool is_R_pot_bending();

    void calibrate_IMU();

    double get_delta_IMU_heading(double prev_IMU_heading);

    double get_IMU_heading();

    void set_alpha(double new_alpha_in_degrees);

    void use_IMU_for_odometry(bool is_IMU_odometry);

    void reset_odom();

    void stop_drive_motors();

    void driver_control(double Yaxis, double Xaxis, double turn);

    void setup();

    void center_on_tower_with_bumper(double angle, bool use_IMU, double timeout = 2000);

    ~DriveTrain();
};

#endif
