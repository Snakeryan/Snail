#ifndef SCORER_H
#define SCORER_H

#include "main.h"


class Scorer
{
    //shared pointer to ensure that the scorer tasks will never outlive your function
    std::shared_ptr<pros::Task> auton_sensors_task{nullptr};
    std::shared_ptr<pros::Task> flywheel_and_indexer_manager_task{nullptr};
    std::shared_ptr<pros::Task> intake_manager_task{nullptr};

    // counters for the balls entering and exiting our loop and upper balls:
    int lower_balls_counted, upper_balls_counted = 0;

    //calibrated value of the light sensor:
    double light_sensor_calibrated_value, prev_upper_light_value, prev_lower_limit_value, upper_prev_time, lower_prev_time = 0;

    //flag for dispensing
    bool dispense_triggered = false;

    //pointer objects of all of the motors that are not part of the drivetrain:
    pros::Motor *intakeleft;
    pros::Motor *intakeright;
    pros::Motor *indexer;
    pros::Motor *flywheel;

    //pointer object of the vision sensor:
    pros::Vision *vision_sensor;

    //pointer objects of all of our vision signatures:
    pros::vision_signature_s_t *BLUE_BALL_SIGNATURE;
    pros::vision_signature_s_t *RED_BALL_SIGNATURE;
    pros::vision_signature_s_t *tower_backboard_signature;

    //pointers to the three-wire sensors:
    pros::ADIDigitalIn *lower_limit_switch;
    pros::ADIAnalogIn *light_sensor;

    void run_lower_limit_switch();

    void run_upper_light_sensor();

    void start_auton_sensors_update_thread();

    void start_indexer_and_flywheel_management_thread();

    void start_intake_manager_thread();

    void manage_indexer_and_flywheel();

    void make_scorer_threads();

public:
    Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::vision_signature_s_t *tower_backboard_signature, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *light_sensor);

    void set_intake(int power);

    void set_flywheel(int flywheel_power);

    void set_indexer(int indexer_power);

    double get_light_calibrated_value();

    void deploy_intakes();

    void wait_until_number_of_uppper_balls_counted(int number_of_balls_passed);

    void wait_until_number_of_lower_balls_counted(int number_of_balls_passed);

    void score_in_goal(int num_balls);

    void score_in_goal_with_light(int num_balls);

    void dispense();

    void reset_balls_counted();

    void stop_motors();

    void setup();

    ~Scorer();
};

#endif