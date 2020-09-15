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
    double prev_upper_light_value, prev_lower_limit_value, upper_prev_time, lower_prev_time, num_balls_to_score, num_balls_to_collect = 0;

    //flag for dispensing
    bool dispense_triggered;

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

    //pointers to the three-wire sensors:
    pros::ADIDigitalIn *lower_limit_switch;
    pros::ADIAnalogIn *upper_counter_light_sensor;

    void manage_intakes();
    /**
 *        makes a lower ball counter with a limit switch (use this to know how many balls have entered the robot)
*/
    void run_lower_limit_switch();

    /**
 *        makes an upper ball counter with a light sensor (use this to know how many balls have exited the robot)
*/
    void run_upper_light_sensor();

    /**
 *        puts run_upper_light_sensor() and run_lower_limit_switch() in a while true loop with a small task delay
*/
    void start_auton_sensors_update_thread();

    /**
 *        will dispence balls when a flag is activated
*/
    void manage_indexer_and_flywheel();

    /**
 *        puts manage_indexer_and_flywheel() in a while true loop with a small task delay 
*/
    void start_indexer_and_flywheel_management_thread();

    /**
 *        puts manage_indexer() in a while true loop with a small task delay 
*/
    void start_intake_manager_thread();

    /**
 *        scores a specified amount of balls using the upper ball counter and the light sensor to ensure that balls do not collide with each other (turns indexers off when the light sensor is detected)
 * \param num_balls
 *        how many balls to shoot
*/
    void score_in_goal_with_light(int num_balls);

public:
    /** 
 * \param intakeleft
 *        address of the left intake's motor
 * \param intakeright
 *        address of the right intake's motor
 * \param indexer
 *        address of the indexers' motor
 * \param flywheel
 *        address of the flywheel's motor
 * \param vision_sensor
 *        address of the vision sensor
 * \param BLUE_BALL_SIGNATURE
 *        address of the vision sensor's blue ball signature
 * \param RED_BALL_SIGNATURE 
 *        address of the vision sensor's red ball signature
 * \param lower_limit_switch
 *        address of the lower_limit_switch
 * \param upper_counter_light_sensor
 *        address of the upper light sensor
*/
    Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *upper_counter_light_sensor);

    /**
 *        makes all of the scorer threads
*/
    void make_scorer_threads();

    /**
 *        sets the speed of the intakes
 * \param power
 *        the power at which to run the intakes
*/
    void set_intakes(int intake_power);

    void score_n_balls(double n_balls = 0);
    /**
 *        sets the flywheel's speed
 * \param flywheel_power
 *        the power at which to run the flywheel
*/
    void set_flywheel(int flywheel_power);

    /**
 *        sets the speed of the indexers
 * \param indexer_power
 *        the power at which to run the indexers
*/
    void set_indexers(int indexer_power);

    /**
 * \return
 *        the current value of the light sensor with the calibrated light sensor's value being zero
*/
    double get_light_calibrated_value();

    /**
 * \return
 *        deploys the intakes using a start and stop method
*/
    void deploy_intakes();

    /**
 *        waits until a certain number of balls have passed the light sensor
 * \param number_of_balls_passed
 *        how many balls to wait to pass the light sensor
*/
    void wait_until_number_of_uppper_balls_counted(int number_of_balls_passed);

    /**
 *        waits until a certain number of balls have passed the lower limit switch
 * \param number_of_balls_passed
 *        how many balls to wait to pass the limit switch
*/
    void wait_until_number_of_lower_balls_counted(int number_of_balls_passed);

    /**
 *        activates the dispensing flag (will automatically be turned off) and dispenses the balls within the robot
*/
    void dispense();

    /**
 *        resets the lower and upper balls counted
*/
    void reset_balls_counted();

    void collect_n_balls(double num_balls_to_collect);

    double get_lower_balls_counted();

    double get_upper_balls_counted();

    void stop_motors();

    void setup();

    ~Scorer();
};

#endif