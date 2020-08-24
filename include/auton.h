#ifndef AUTON
#define AUTON

extern int lower_balls_counted;
extern int upper_balls_counted;

void set_intake(int power);
double get_light_calibrated_value();
void set_flywheel_and_indexer(int flywheel_power, int indexer_power);
void run_auton_sensors();
void manage_flywheel();
void debug_autonomous();
void go_home();
void stop_drive_motors();
void run_homerow(void);
void run_skills(void);
void autonomous();
void wait_until_number_of_lower_balls_counted(int number_of_balls_passed);

#endif