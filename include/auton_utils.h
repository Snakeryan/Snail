#ifndef AUTON_UTILS
#define AUTON_UTILS

#include <pthread.h>


class AutonUtils
{



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
    void update();
    void start_update_thread();
    double rad_to_deg(double rad);
    double deg_to_rad(double deg);
    double compute_alpha(double right_encoder_distance, double left_encoder_distance);
    

   
    public:
    void make_update_thread();
    AutonUtils(double encoder_wheel_radius, double wL, double wR, double wM, pros::Motor* FL, pros::Motor* FR, pros::Motor* BL, pros::Motor* BR, pros::ADIEncoder* encoderL, pros::ADIEncoder* encoderR, pros::ADIEncoder* encoderM);
    double get_alpha_in_degrees();
    double get_globalX();
    double get_globalY();
    double get_left_encoder_distance();
    double get_right_encoder_distance();
    double get_middle_encoder_distance();
    double turn_to_point(double X2, double Y2);
    double move_distance(double X2, double Y2);
    ~AutonUtils();
    
};


void setTurn(int turn);

void turn(double target, int power);


double compute_error(double target, double current_angle);

void point_turn_PID(int target, const double Kp, const double Ki, const double Kd, bool do_once = true);

#endif
