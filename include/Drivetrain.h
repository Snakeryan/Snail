/******************************************************************************/
/**                General methods for X-drive drivetrains                   **/
/******************************************************************************/

#ifndef drivetrain_H
#define drivetrain_H

#include <pthread.h>
#include <memory>
#include "main.h"

class Drivetrain
{

    // Task that updates the odometry variables:
    pros::Mutex update_odometry_mutex;

    double encoder_wheel_radius, wL, wR, wM, globalX, globalY, alpha, prev_alpha, IMU_heading, prev_IMU_heading, prev_left_encoder_distance, delta_left_encoder_distance, prev_right_encoder_distance, delta_right_encoder_distance, prev_middle_encoder_distance, delta_middle_encoder_distance = 0;

    // flag for running odometry with and without the IMU:
    bool is_IMU_odometry;

    //shared pointer to ensure that the update_odometry task will never outlive the function
    std::shared_ptr<pros::Task>
        odometry_update_task{nullptr};

    // motor/sensors:
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

    /**
 *        applies motor power to the X-drive motors
 * \param T
 *        the direction the robot will face
 * \param S
 *        how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *        ratio between how much motor power to apply to rotational versus translational speeds
 * \param K_constant
 *        a constant to multiply R by (has default of 1)
*/
    void run_Xdrive(double T, double S, double R, double K_constant = 1);

    /**
 *        uses this formula: (delta_left_distance - delta_right_distance) / (wL + wR)
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
 *        uses this formula: (2 * sin(delta_alpha / 2)) * (wR + (delta_right_distance /delta_alpha))
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \param delta_right_distance
 *        the change in the value of the distance of the right encoder (takes inches)
 * \return 
 *        the change in the robot's local Y coordinate/position 
*/
    double compute_delta_Dly(double delta_alpha, double delta_right_distance);

    /**
 *        uses this formula: (2 * sin(delta_alpha / 2)) * ((delta_middle_distance / delta_alpha) + wM)
 * \param delta_alpha
 *        the change in the robot's heading (takes radians)
 * \param delta_middle_distance
 *        the change in the value of the distance of the middle encoder (takes inches)
 * \return 
 *        the change in the local X coordinate/position
*/
    double compute_delta_Dlx(double delta_alpha, double delta_middle_distance);

    /**
 *        uses this formula: (Dlx * cos(delta_alpha/2 + prev_alpha)) + (Dly * sin(delta_alpha/2 + prev_alpha))
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
 *        uses this formula: (Dly * cos(delta_alpha/2 + prev_alpha)) - (Dlx * sin(delta_alpha/2 + prev_alpha))
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
 *        uses this formula: -cos(T + pi/4)
 * \param T
 *        the direction the robot will face
 * \return 
 *        value calculated for smooth movements of the robot's front right and back left wheels
*/
    double compute_P1(double T);

    /**
 *        uses this formula: sin(T + pi/4)
 * \param T
 *        the direction the robot will face
 * \return 
 *        value calculated for smooth movements of the robot's front left and back right wheels
*/
    double compute_P2(double T);

    /** 
 *        uses this formula: MAX(abs(P1), abs(P2)) / S
 * \param P1
 *        value calculated for smooth movements of the robot's front right and back left wheels
 * \param P2
 *        value calculated for smooth movements of the robot's front left and back right wheels
 * \param S
 *        how much motor power to apply to translational movement (if S = 1 more translation and S = 0 is less translational)
 * \return 
 *        the larger of the two values of P1 and P2 divided by S
*/
    double compute_s(double P1, double P2, double S);

    /** 
 *       uses this formula: (P2 / s) * (1 - abs(R)) + R * K_constant
 * \param P2
 *       value calculated for smooth movements of the robot's front left and back right wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to multiply R by
*/
    void compute_FL_motor_speed(double P2, double s, double K_constant, double R, double multiplier);

    /** 
 *       uses this formula: ((P1 / s) * (1 - abs(R)) - R * K_constant) * -1
 * \param P1
 *       value calculated for smooth movements of the robot's front right and back left wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to multiply R by
*/
    void compute_FR_motor_speed(double P1, double s, double K_constant, double R, double multiplier);

    /** 
 *        uses this formula: ((P1 / s) * (1 - abs(R)) + R * K_constant)
 * \param P1
 *        value calculated for smooth movements of the robot's front right and back left wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to multiply R by
*/
    void compute_BL_motor_speed(double P1, double s, double K_constant, double R, double multiplier);

    /** 
 *        uses this formula: ((P2 / s) * (1 - abs(R)) - R * K_constant) * -1
 * \param P2
 *        value calculated for smooth movements of the robot's front left and back right wheels
 * \param s
 *       the larger of the two values P1 and P2 (ensures that the largest P value is equal to 1 
 * \param S
 *       how much motor power to apply to translational (if S = 1 more translation and S = 0 is less translational)
 * \param R
 *       ratio between the rotational and translational errors (tells how much motor power to apply to each)
 * \param multiplier
 *       a constant to multiply R by
*/
    void compute_BR_motor_speed(double P2, double s, double K_constant, double R, double multiplier);

    /** 
 *        main function for odometry, as it is where all of the variables are updated and absolute coordinates are calculated
*/
    void update_odometry();

    /**
 *        uses this formula: (left_encoder_distance - right_encoder_distance) / (wL + wR)
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
 *        always returns a value between 0-2π and accepts wrapped degree values (can put in IMU values)
 * \param target
 *        the angle to turn to (in radians)
 * \param current_angle
 *        the current heading of the robot (in radians)
 * \return 
 *        the error between the target angle and the current angle
*/
    double compute_angle_error(double target, double current_angle);

    /** 
 *        sets up some of the sensors used within this class
*/
    void setup_sensors();

    /**
 *        calls start_odometry_update_thread() to create a seperate task for updating the odometry variables
*/
    void make_odometry_update_thread();

public:
    /** 
 *        this puts the update_odometry function into a while (true) loop with a ten miliseconds delay
*/
    void start_odometry_update_thread();

    /** 
 *        calls all of the motion algorithm functions and allows for smooth translations and rotations (requires X-drive)
 * \param tX
 *        X coordinate to travel to
 * \param tY
 *        Y coordinate to travel to
 * \param target_angle_in_degrees
 *        the angle to turn to
 * \param point_type
 *        what type of point to use: 1 = waypoint (most speed least accuracy), 2 = hybrid point/default point (second most speed more accuracy), anything else = destination point (least speed most accuracy)
 * \param is_waypoint
 *        if the point you are traveling to is a waypoint to get to another point, set this parameter to true (will only move translationally)
 * \param rotational_KP
 *        what to multiply R by (the default value 1 will turn while moving and not keep a stable angle and a value of 3 will turn then move and keep a more stable angle)
 * \param trigger
 *        triggers a function that will happen at a certain distance away from the target point (can pass in a lambda and pass in NULL if trying to get to other parameters and not wanting to pass in a function)
 * \param trigger_distance
 *        the distance away from the target coordinate to activate the function passed in (put 0 to do nothing)
 * \param timeout
 *       the maximum amount of time the robot can spend driving to a point (has default of 10000 milliseconds)
*/
    void drive_to_point(double tX, double tY, double target_angle_in_degrees, int point_type = 2, double rotational_KP = 1, const std::function<void()> &trigger = 0, double trigger_distance = 3, double timeout = 10000);

    /** 
 *        will change the global coordinates and the odometry angle to given values
 * \param new_X
 *        new X coordinate
 * \param new_Y
 *        new Y coordinate
 * \param new_alpha_in_degrees
 *        new heading
*/
    void set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees);

    /** 
 * \param encoder_wheel_radius
 *        the radius of the tracking wheels (takes inches and if using the smallest vex wheels, the radius is 1.375 inches)
 * \param wL
 *        the distance from the center of the robot to the center of the left tracking wheel (if equal to wR, use to experimentally find: ((drivetrain.get_left_encoder_distance() - drivetrain.get_right_encoder_distance()) / (20 * pi)) / 2 SPIN TEN TIMES then put in for wL and wR
 * \param wR
 *        the distance from the center of the robot to the center of the right tracking wheel (if equal to wL, use to experimentally find: ((drivetrain.get_left_encoder_distance() - drivetrain.get_right_encoder_distance()) / (20 * pi)) / 2 SPIN TEN TIMES then put in wR and wL
 * \param wM
 *        the distance from the center of the robot to the center of the middle/back tracking wheel (if equal to wL and wR, use to experimentally find: ((drivetrain.get_left_encoder_distance() - drivetrain.get_right_encoder_distance()) / (20 * pi)) / 2 SPIN TEN TIMES then put in for wM, wL, and wR
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
 * \param vision_sensor
 *        the address of the vision sensor
 * \param left_pot
 *        the address of the left potentiometer
 * \param right_pot
 *        the address of the right potentiometer
*/
    Drivetrain(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor *FL, pros::Motor *FR, pros::Motor *BL, pros::Motor *BR, pros::ADIEncoder *encoderL, pros::ADIEncoder *encoderR, pros::ADIEncoder *encoderM, pros::Vision *vision_sensor, pros::Imu *IMU, pros::ADIAnalogIn *left_pot, pros::ADIAnalogIn *right_pot);

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
 *        PID loop for turning to a specific angle
 * \param target
 *        the target angle to turn to (in degrees)
 * \param use_IMU
 *        set true to use the IMU
 * \param Kp
 *        the Proportional control constant to multiply by (has default of 37.5, but recommend changing) 
 * \param Ki
 *        the integral control constant to multiply by (has default of 0.7, but recommend changin) 
 * \param Kd
 *        the derivative control constant to multiply by (has default of -0, as this constant is supposed to be negative)
*/
    void point_turn_PID(double target, bool use_IMU = false, const double Kp = 30, const double Ki = 1.1, const double Kd = -65);

    /** 
 *        uses a PID loop to turn to a point on the coordinate system (coordinates are in inches)
 * \param X2
 *        the X coordinate to turn to
 * \param Y2
 *        the Y coordinate to turn to
*/
    void turn_to_point(double X2, double Y2);

    /** 
 *        sets the X-drive motors at a specified value (put in value between -127 to 127 and DO NOT USE FOR DRIVER CONTROL)
 * \param FL_motor_power
 *        the power to apply to the front left motor
 * \param FR_motor_power
 *        the power to apply to the front right motor
 * \param BL_motor_power
 *        the power to apply to the back left motor 
 * \param BR_motor_power
 *        the power to apply to the back right motor
*/
    void set_motors(double FL_motor_power, double FR_motor_power, double BL_motor_power, double BR_motor_power);

    /**
 * \param encoder_readings_to_average
 *        the number of consecutive delta encoder wheel readings that have to be equal to zero         
 * \return
 *        boolean testing if the delta encoder wheel readings all equal zero
*/
    bool collision_detected(const int encoder_readings_to_average = 3);
    /**
 * \return
 *        boolean seeing if the left potentiometer is bending
*/
    bool is_L_pot_bending();

    /**
 * \return
 *        boolean seeing if the righ potentiometer is bending
*/
    bool is_R_pot_bending();

    /**
 *        calibrates an Inertial Measurment Unit (IMU) (it is essential to calibrate the IMU before getting readings)
*/
    void calibrate_IMU();

    /**        
 * \return
 *        the odometry calculated IMU heading 
*/
    double get_IMU_heading();

    /**
 *        uses this formula: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert into degrees
 * \return 
 *        the degree version of the radian value (method also constrains this version between 0-360)
*/
    double convert_rad_to_deg_wraped(double rad);

    /**
 *        uses this formula: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert into degrees
 * \return 
 *        the degree version of the radian value 
*/
    double convert_rad_to_deg(double rad);

    /**
 *        uses this formula: (deg * pi)/180
 * 
 * \param deg
 *        the degree value that you want to convert into radians
 * \return 
 *        the radian version of the degree value (does not wrap the radians between 0-2π)
*/
    double convert_deg_to_rad(double deg);

    /**
 * \return 
 *        the radian version of the heading of the robot (this is constrained from 0-2π)
*/
    double get_constrained_alpha();

    /**
 *        assigns a new value to the angle used for odometry
 * \param new_alpha_in_degrees
 *        the new angle to be assigned to the odometry calculated angle       
*/
    void set_alpha(double new_alpha_in_degrees);

    /**
 *        will make the odometry calculations based off of the IMU instead of the odometry calculated angle
*/
    void use_IMU_for_odometry(bool is_IMU_odometry);

    /**
 *        will reset the global/absolute coordinates, odometry angle, and the calculated IMU angle to zero
*/
    void reset_odom();

    /**
 *        sets the X-drive motors to zero
*/
    void stop_drive_motors();

    /** 
 *        runs X-drive through controller movements 
 * \param Yaxis
 *        what joystick direction you want moving forward and backward to be
 * \param Xaxis
 *        what joystick direction you want moving right and left to be
 * \param turn
 *        what joystick direction you want turning to be (CANNOT BE THE SAME AS ONE OF THE OTHER PARAMETERS)
*/
    void driver_control(double Yaxis, double Xaxis, double turn);

    /** 
 *        sets up this classes sensors and creates the update thread for odometry 
*/
    void setup();

    /** 
 *        centers on a game tower by using the same algorithms as drive_to_point (allows the robot to keep a constant heading while moving) 
 * \param angle
 *        the angle the robot will maintain while drivingf
 * \param use_IMU
 *        boolean for using the IMU (true) or the odometry calculated angle (false)
 * \param timeout
 *        the maximum amount of time the function can run for
 * \param use_pots
 *        will use potentiometers to right the robot's position if set to true
*/
    void center_on_tower_with_bumper(double target_angle, bool use_IMU, double timeout = 2000, bool use_pots = false);

    /** 
 *        a destructor to ensure that the drivetrain tasks never outlive the object
*/
    ~Drivetrain();
};

#endif
