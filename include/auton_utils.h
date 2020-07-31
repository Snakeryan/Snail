#ifndef AUTON_UTILS
#define AUTON_UTILS

void setTurn(int turn);

void turn(double target, int power);


double compute_error(double target, double current_angle);

void point_turn_PID(int target, const double Kp, const double Ki, const double Kd);

#endif
