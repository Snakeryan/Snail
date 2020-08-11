#ifndef AUTON_UTILS
#define AUTON_UTILS

/******************************************************************************/
/**           General methods for smooth, autonomous movements               **/
/**                                                                          **/
/**       These methods allow for absolute tracking and positioning          **/
/******************************************************************************/



#include <pthread.h>


class AutonUtils
{
    

   // Task that updates the odometry functions:
    pros::Mutex update_mutex;

    // all data members of the AutonUtils class:
    double encoder_wheel_radius, wL, wR, wM, globalX, globalY, alpha, prev_alpha, prev_left_encoder_distance, prev_right_encoder_distance, prev_middle_encoder_distance;

    //shared pointer to ensure that the update task(s) will never outlive your function
    std::shared_ptr<pros::Task> task{nullptr};

    // pointers to allow for your motor/sensor parameters to be accessed:
    pros::Motor* FL;
    pros::Motor* FR;
    pros::Motor* BL;
    pros::Motor* BR;
    pros::ADIEncoder* encoderL;
    pros::ADIEncoder* encoderR;
    pros::ADIEncoder* encoderM;

/**
 * 
 * \note 
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
 * \note 
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
 * \note 
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
 * \note 
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
 * \note 
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
 * \note 
 *        this is the formula used by this method: -cos(T + pi/4)
 * \param T
 *        How much the current coordinate plane needs to be offset by to fix the robot's directional perspective
 * \return 
 *        value calculated for smooth movements of the robot's front right and back left wheels
*/    
    double compute_P1(double T);

/**
 * \note 
 *        this is the formula used by this method: sin(T + pi/4)
 * \param T
 *        How much the current coordinate plane needs to be offset by to fix the robot's directional perspective
 * \return 
 *        value calculated for smooth movements of the robot's front left and back right wheels
*/ 
    double compute_P2(double T);

/**
 * \note 
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
 * \note 
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
    void compute_FL_motor_speed(double P2, double s, double S, double R, double multiplier, double use_motor = 1);

/**
 * \note 
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
    void compute_FR_motor_speed(double P1, double s, double S, double R, double multiplier, double use_motor = 1);

/**
 * \note 
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
    void compute_BL_motor_speed(double P1, double s, double S, double R, double multiplier, double use_motor = 1);

/**
 * \note 
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
    void compute_BR_motor_speed(double P2, double s, double S, double R, double multiplier, double use_motor = 1);

/**
 * \param void 
 *        main funcition for odometry, as it is where all of the variables are updated and absolute coordinates are calculated
*/ 
    void update();

/**
 * \param void 
 *        this puts the update function into a while(true) loop with a very small delay of ten miliseconds 
*/ 
    void start_update_thread();



/**
 * \note 
 *        this is the formula used by this method: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert to degrees
 * \return 
 *        the degree version of the radian value (method also constrains this version between 0-360)
*/
    double rad_to_deg_wraped(double rad);

/**
 * \note 
 *        this is the formula used by this method: (rad * 180)/pi
 * \param rad
 *        the radian value that you want to convert to degrees
 * \return 
 *        the degree version of the radian value 
*/
    double rad_to_deg(double rad);

/**
 * \note 
 *        this is the formula used by this method: (deg * pi)/180
 * \param deg
 *        the degree value that you want to convert to radians
 * \return 
 *        the radian version of the degree value (does not wrap the radians between 0-2π)
*/
    double deg_to_rad(double deg);

/**
 * \note 
 *        this is the formula used by this method: (left_encoder_distance - right_encoder_distance) / (wL + wR)
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
    void set_turn(int turn);

/**
 * \param turn
 *        the speed at which you want the robot to turn (12 is the maximum speed)
*/
    double compute_error(double target, double current_angle);

/**
 * \return 
 *        the radian version of the heading of the robot (this is constrained from 0-2π)
*/
    double get_constrained_alpha();

/**
 * \note  
*/
    public:
/**
 * \param void 
 *        this function calls start_update_thread() to create a seperate task for updating the odometry variables
*/ 
    void make_update_thread();

/**
 * \param turn
 *        the speed at which you want the robot to turn (12 is the maximum speed)
*/
    
    void drive_to_point(double tX, double tY, double target_angle_in_degrees, bool use_precise_turn, bool is_waypoint);
    void set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees);
    AutonUtils(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor* FL, pros::Motor* FR, pros::Motor* BL, pros::Motor* BR, pros::ADIEncoder* encoderL, pros::ADIEncoder* encoderR, pros::ADIEncoder* encoderM);
    double get_alpha_in_degrees();
    double get_globalX();
    double get_globalY();
    double get_left_encoder_distance();
    double get_right_encoder_distance();
    double get_middle_encoder_distance();
    void point_turn_PID(double target, const double Kp = 37.5, const double Ki = .7, const double Kd = -0, bool use_IMU = false, bool do_once = false);
    void turn_to_point(double X2, double Y2);
    void move_distance(double X2, double Y2);
    ~AutonUtils();
};




#endif
