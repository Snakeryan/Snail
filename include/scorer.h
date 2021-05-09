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
    int lower_balls_counted, upper_balls_counted, dispense_balls_counted, middle_balls_counted = 0;

    double prev_light_value, prev_upper_light_value, prev_dispense_light_value, prev_lower_light_value, prev_middle_light_value, upper_prev_time, dispense_prev_time, prev_time, lower_prev_time, middle_prev_time, num_balls_to_score, num_balls_to_collect, intake_speed, flywheel_speed, indexer_speed, num_balls_to_dispense, num_balls_to_dispense_through_intakes, time_taken_per_ball, balls_to_score = 0;

    bool is_ball_collect_point, is_more_balls;
    //pointer objects of all of the motors that are not part of the drivetrain:
    pros::Motor *intakeleft;
    pros::Motor *intakeright;
    pros::Motor *indexer;
    pros::Motor *flywheel;

    //pointer object of the vision sensor:
    pros::Vision *vision_sensor;

    // flag for deploying the bumper's fangs
    bool position_fangs, position_intakes, is_center_tower;

    //pointer objects of vision signatures:
    pros::vision_signature_s_t *BLUE_BALL_SIGNATURE;
    pros::vision_signature_s_t *RED_BALL_SIGNATURE;

    //pointers to the three-wire sensors:
    pros::ADIAnalogIn *lower_counter_light_sensor;
    pros::ADIAnalogIn *upper_counter_light_sensor;
    pros::ADIAnalogIn *middle_light_sensor;
    pros::ADIAnalogIn *dispense_counter_light_sensor;

    /**
 *        manages collecting and dispensing through the intakes
*/
    void manage_intakes();

    /**
 *        makes a lower ball counter with a light sensor (use this to know how many balls have entered the robot)
*/
    void run_lower_light_sensor();

    /**
 *        makes an upper ball counter with a light sensor (use this to know how many balls have exited the robot's loop)
*/
    void run_upper_light_sensor();

    /**
 *        will eventually implement this to eliminate redundancies
*/
    void setup_light_counter(double light_sensor_threshold, double light_calibrated_value, double counter);

     /**
 *        makes a middle counter with a light sensor (use this to turn off the indexers when the ball reaches the flywheel)
*/
    void run_middle_light_sensor();

    /**
 *        makes a dispense counter with a light sensor (use this to know how many balls have exited the robot's dispenser)
*/
    void run_dispense_light_sensor();

    /**
 *        puts run_upper_light_sensor() and run_lower_limit_switch() in a while true loop with a small task delay
*/
    void start_auton_sensors_update_thread();

    /**
 *        manages scoring, dispensing through the dispenser, and turning on the indexers to collect balls
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
 *        scores a specified amount of balls using the upper ball counter and
 * the light sensor to ensure that balls do not collide with each other (turns
 * indexers off when a ball is detected by the upper light sensor)
 * \param num_balls
 *        how many balls to shoot
 * \param time_per_ball
 *        max time robot can take to score a ball
 * \param is_more_balls
 *        use if scoring more than one ball
 * \param flywheel_speed
 *        the speed to spin the flywheel
 * \param indexer_speed
 *        the speed to spin the flywheel
*/
    void score_in_goal_with_light(int num_balls, int time_per_ball, bool is_more_balls, double flywheel_speed, double indexer_speed);

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
 * \param lower_counter_light_sensor
 *        address of the lower_counter_light_sensor
 * \param upper_counter_light_sensor
 *        address of the upper light sensor
 * \param dispense_counter_light_sensor
 *        address of the dispense light sensor
 * \param middle_light_sensor
 *        address of the middle light sensor
*/
    Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::ADIAnalogIn *lower_counter_light_sensor, pros::ADIAnalogIn *upper_counter_light_sensor, pros::ADIAnalogIn *dispense_counter_light_sensor, pros::ADIAnalogIn *middle_light_sensor);

    /**
 *        makes all of the scorer threads (auton sensors thread, flywheel and indexer management thread, and intake management thread)
*/
    void make_scorer_threads();

    /**
 *        sets the speed of the intakes
 * \param power
 *        the power at which to run the intakes (between -127 to 127)
*/
    void set_intakes(int intake_power);

    /**
 *        scores a specified amount of balls
 * \param n_balls
 *        how many balls to score (default of -1 will score the number of balls collected)
 * \param time_taken_per_ball
 *        how long the robot should take to score a singe ball (default is 1250)
 * \param is_more_balls
 *        use if scoring more than one ball
 * \param flywheel_speed
 *        the speed to spin the flywheel (default is 127)
 * \param indexer_speed
 *        the speed to spin the flywheel (default is 127)
*/
    void score_n_balls(double num_balls_to_score = -1, double time_taken_per_ball = 1250, bool is_more_balls = false, double flywheel_speed = 127, double indexer_speed = 127);

    /**
 *        sets the speed of the flywheel
 * \param flywheel_power
 *        the power at which to run the flywheel (between -127 to 127)
*/
    void set_flywheel(int flywheel_power);

    /**
 *        sets the speed of the indexers
 * \param indexer_power
 *        the power at which to run the indexers (between -127 to 127)
*/
    void set_indexers(int indexer_power);

    /**
 * \return
 *        the current value of the upper light sensor with the calibrated light sensor's value being zero
*/
    double get_upper_light_calibrated_value();

    /**
 * \return
 *        the current value of the lower light sensor with the calibrated light sensor's value being zero
*/
    double get_lower_light_calibrated_value();

    /**
 * \return
 *        the current value of the middle light sensor with the calibrated light sensor's value being zero
*/
    double get_middle_light_calibratred_value();

    /**
 * \return
 *        the current value of the dispense light sensor with the calibrated light sensor's value being zero
*/
    double get_dispense_light_calibrated_value();

    /**
 *        waits until a certain number of balls have passed the light sensor
 * \param number_of_balls_passed
 *        how many balls to wait to pass the light sensor (default will score the number of balls collected)
*/
    void wait_until_number_of_upper_balls_counted(int number_of_balls_passed = -1);

    /**
 *        waits until a certain number of balls have passed the lower limit switch
 * \param number_of_balls_passed
 *        how many balls to wait to pass the lower limit switch
*/
    void wait_until_number_of_lower_balls_counted(int number_of_balls_passed);

    /**
 *        will dispense a specified number of balls (is asynchronous/non-blocking)
 * \param num_balls_to_dispense
 *        the number of balls to dispense
 * \param is_intake_dispense
 *        will dispense out of intakes instead of the dispenser if set to true
*/
    void dispense_n_balls(double num_balls_to_dispense, bool is_intake_dispense = false);

    /**
 *        waits until a certain amount of balls have passed the lower limit switch or the dispense light sensor 
 * \param num_balls_to_dispense
 *        the number of balls to wait to pass either the lower limit switch or the dispense light sensor 
 * \param is_intake_dispense
 *        will wait for balls to pass the lower limit switch instead of the dispense light sensor
*/
    void wait_until_number_of_balls_dispensed(int num_balls_to_dispense, bool is_intake_dispense = false);

    /**
 *        resets the lower, upper, and dispensed balls counted
*/
    void reset_balls_counted();

    /**
 *        collects a specified amount of balls (this code is asynchronous/non-blocking)
 * \param num_balls_to_collect
 *        how many balls to collect
 * \param intake_speed
 *        speed to set the intakes (default is 127)
 * \param is_center_tower
 *        set to true when running the center tower (default is false)
 
*/
    void collect_n_balls(double num_balls_to_collect, double intake_speed = 127, bool is_center_tower = false);

    /**
 * \return
 *        the current value of balls_to_score
*/
    double get_balls_to_score();

    /**
 * \return
 *        the current value of lower_balls_counted
*/
    double get_lower_balls_counted();

        /**
 * \return
 *        the current value of middle_balls_counted
*/
    double get_middle_balls_counted();


    /**
 * \return
 *        the current value of upper_balls_counted
*/
    double get_upper_balls_counted();

    /**
 * \return
 *        the current value of dispense_balls_counted
*/
    double get_dispense_balls_counted();

    /**
 *        will count balls entering the system to later be scored in a tower
 * \param is_ball_count_point
 *        if set to true, the counter will count (only set to true when about to collect a ball to later be scored and not dispensed)
*/
    void count_balls_to_score(bool is_ball_collect_point);

    /**
 *        resets the balls_to_score to zero (call this when resetting on a game tower)
*/
    void reset_balls_to_score();

    /**
 *        turns off the intakes, indexers, and flywheel
*/
    void stop_motors();

    /**
 *        creates the scorer threads 
*/
    void setup();

    /**
 *        destructor to ensure that the scorer threads do not outlive the object
*/
    ~Scorer();
};

#endif