#ifndef AUTON
#define AUTON

void set_intake(int power);

void set_flywheel_and_indexer(int flywheel_power, int indexer_power);
void run_auton_sensors();
void manage_flywheel();
void debug_autonomous();
void go_home();
void stop_drive_motors();
void run_homerow(void);
void run_skills(void);
void autonomous();

#endif