#ifndef AUTON_UTILS
#define AUTON_UTILS

#include <pthread.h>


class AutonUtils
{

    pros::Mutex update_mutex;

    double encoder_wheel_radius, wL, wR, wM, globalX, globalY, alpha, prev_alpha, prev_left_encoder_distance, prev_right_encoder_distance, prev_middle_encoder_distance;
    std::shared_ptr<pros::Task> task{nullptr};
    pros::Motor* FL;
    pros::Motor* FR;
    pros::Motor* BL;
    pros::Motor* BR;
    pros::ADIEncoder* encoderL;
    pros::ADIEncoder* encoderR;
    pros::ADIEncoder* encoderM;

    double compute_delta_alpha(double delta_right_distance, double delta_left_distance);
    double compute_delta_Dly(double delta_alpha, double delta_right_distance);
    double compute_delta_Dlx(double delta_alpha, double delta_middle_distance);
    double compute_delta_globalX(double dlX, double dlY, double delta_alpha);
    double compute_delta_globalY(double dlX, double dlY, double delta_alpha);
    double compute_P1(double T);
    double compute_P2(double T);
    double compute_s(double P1, double P2, double S);
    void compute_FL_motor_speed(double P2, double s, double S, double R);
    void compute_FR_motor_speed(double P1, double s, double S, double R);
    void compute_BL_motor_speed(double P1, double s, double S, double R);
    void compute_BR_motor_speed(double P2, double s, double S, double R);
    void update();
    void start_update_thread();
    double rad_to_deg_wraped(double rad);
    double deg_to_rad(double deg);
    double compute_alpha(double right_encoder_distance, double left_encoder_distance);
    void set_turn(int turn);
    void turn(double target, int power);
    double compute_error(double target, double current_angle);
    double get_constrained_alpha();

    public:
    double rad_to_deg(double rad);
    void drive_to_point(double tX, double tY, double target_angle_in_degrees);
    void make_update_thread();
    void set_current_global_position(double new_X, double new_Y, double new_alpha_in_degrees);
    AutonUtils(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor* FL, pros::Motor* FR, pros::Motor* BL, pros::Motor* BR, pros::ADIEncoder* encoderL, pros::ADIEncoder* encoderR, pros::ADIEncoder* encoderM);
    double get_alpha_in_degrees();
    double get_globalX();
    double get_globalY();
    double get_left_encoder_distance();
    double get_right_encoder_distance();
    double get_middle_encoder_distance();
    void point_turn_PID(double target, const double Kp = 45, const double Ki = 0, const double Kd = -0, bool use_IMU = false, bool do_once = false);
    void turn_to_point(double X2, double Y2);
    void move_distance(double X2, double Y2);
    ~AutonUtils();
};




#endif
