#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "globals.h"

int lower_balls_counted = 0;
int upper_balls_counted = 0;

void set_intake(int power)
{
    intakeleft = power;
    intakeright = power;
}

void set_flywheel_and_indexer(int flywheel_power, int indexer_power)
{
    flywheel = flywheel_power;
    indexer = indexer_power;
}

double light_sensor_calibrated_value = 0;

double get_light_calibrated_value()
{
    return light_sensor.get_value() - light_sensor_calibrated_value;
}

void run_auton_sensors()
{
    light_sensor_calibrated_value = light_sensor.calibrate();
    // Lower Limit Switch
    const double delay_time = 100;
    double prev_lower_limit_value = lower_limit_switch.get_value();
    double lower_prev_time = pros::millis();

    // Upper Light Sensor
    const double light_sensor_threshold = -500;
    double prev_light_value = get_light_calibrated_value();
    double upper_prev_time = pros::millis();

    while (true)
    {

        if ((lower_limit_switch.get_value() == 1 && prev_lower_limit_value == 0) && abs(lower_prev_time - pros::millis()) > delay_time)
        {
            lower_balls_counted++;
            lower_prev_time = pros::millis();
        }

        prev_lower_limit_value = lower_limit_switch.get_value();
        // -600 -> 0 crossing detection system, prev = -600, current = 0, threnshold = -500
        if ((get_light_calibrated_value() > light_sensor_threshold && prev_light_value < light_sensor_threshold) && abs(upper_prev_time - pros::millis()) > delay_time)
        {
            upper_balls_counted++;
            upper_prev_time = pros::millis();
        }

        prev_light_value = get_light_calibrated_value();

        // pros::lcd::set_text(3, "left: " + std::to_string(autonutils.get_left_encoder_distance()));
        // pros::lcd::set_text(4, "right: " + std::to_string(autonutils.get_right_encoder_distance()));

        pros::lcd::set_text(6, "upper balls counted: " + std::to_string(upper_balls_counted));
        pros::lcd::set_text(5, "light: " + std::to_string((get_light_calibrated_value())));
        pros::lcd::set_text(7, "lower balls counted: " + std::to_string((lower_balls_counted)));

        pros::Task::delay(10);
    }
}

void debug_autonomous()
{
    while (true)
    {
        pros::lcd::set_text(1, "x, y: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
        pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));

        pros::Task::delay(20);
    }
}

void go_home()
{
    while (true)
    {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            autonutils.drive_to_point(0, 0, 0, true, false);
        }
    }
}

void stop_drive_motors()
{
    FL = 0;
    FR = 0;
    BL = 0;
    BR = 0;
}

void run_homerow()
{
    // pros::Task autonomous_debugger(debug_autonomous);

    set_intake(127);
    autonutils.drive_to_point(0, -15, 45, false, true);

    // First Goal
    autonutils.drive_to_point(16.2, -6.7, 46, false, false);
    auto_sort_balls = true;
    stop_drive_motors();

    set_flywheel_and_indexer(127, 127);
    while (lower_balls_counted < 3)
    {
        pros::delay(50);
    }
    set_intake(-31);
    pros::delay(500);

    // Waypoint to second goal
    // auto_sort_balls = false;
    set_flywheel_and_indexer(0, 0);
    set_intake(0);
    autonutils.drive_to_point(-37.1, -23.7, 0, false, true);
    autonutils.drive_to_point(-37.5, -9.6, 0, false, false);
    stop_drive_motors();

    go_home();
}

void wait_until_number_of_uppper_balls_counted(int number_of_balls_passed)
{
    while (upper_balls_counted < number_of_balls_passed)
    {
        pros::delay(20);
    }
}

void wait_until_number_of_lower_balls_counted(int number_of_balls_passed)
{
    while (lower_balls_counted < number_of_balls_passed)
    {
        pros::delay(20);
    }
}

// function score_in_goal(num_balls):
// while (upperballscounted < target)):
// - activate flywheel
// - activate indexer and if upper_limit_switch is not pressed
// - else: stop indexer; delay 20

//

//if upper_limit is pressed: indexer = 0; pros::delay(50);
//else indexer = 127;

void score_in_goal(int num_balls)
{
    flywheel = 127;
    pros::delay(100);
    indexer = 127;
    while (upper_balls_counted < num_balls)
    {
        pros::delay(10);
    }
    pros::delay(250);
    flywheel = 0;
    indexer = 0;
}

void score_in_goal_with_light(int num_balls)
{
    flywheel = 127;
    indexer = 127;
    while (upper_balls_counted < num_balls)
    {
        if (get_light_calibrated_value() < -500)
        {
            indexer = -100;
            pros::delay(50);
        }
        else
        {
            indexer = 127;
        }
        pros::delay(10);
    }
    pros::delay(100);
    flywheel = 0;
    indexer = 0;
}

void test_mode()
{
    set_intake(127);
    score_in_goal_with_light(2);
    set_intake(0);
}

void deploy_intakes()
{
    set_intake(127);
    pros::delay(200);
    set_intake(0);
    pros::delay(200);
    set_intake(127);
}

void run_skills()
{
    autonutils.set_current_global_position(0, 0, 0);
    // deploy_intakes();
    set_intake(127); 
    autonutils.drive_to_point(0, 13.72, 0, false, true);
    autonutils.drive_to_point(20.10, 6.70, 135.5, false, false, NULL, 0, 3000);
    stop_drive_motors();
    // Score 2 red balls:
    score_in_goal_with_light(2);

    // Store the 2 blue balls:
    indexer = 127;
    wait_until_number_of_lower_balls_counted(3);
    set_intake(-63);
    pros::delay(50);
    set_intake(0);

    //WAYPOINT to tower two:
    autonutils.drive_to_point(7.54, 29.62, 200, false, true, []() { dispense_triggered = true; }, 15);

    // dispense the two blue balls and collect a red ball:
    set_intake(127);
    // autonutils.drive_to_point(-1.88, 45.36, 329.9, false, true);
    
    autonutils.drive_to_point(-2.15, 49.93, 329.7, false, false, []() { indexer = 127; flywheel = 0; }, 3);

    autonutils.drive_to_point(-11.01, 62.10, 0, false, false);

    //drive and turn to tower two:
    autonutils.turn_to_point(16.81, 60.97);
    indexer = 127;
    autonutils.drive_to_point(16.81, 60.97, 91.34, false, false, [](){flywheel = -127;}, 20, 2000);
    stop_drive_motors();

    //reset the number of balls counted:
    lower_balls_counted = 0;
    upper_balls_counted = 0;

    //score in tower two:

    score_in_goal_with_light(1);
    set_intake(-63);
    pros::delay(50);
    set_intake(0);

    //WAYPOINT to tower three:
    autonutils.drive_to_point(-1.40, 110.37, 52, false, true);

    //drive to tower three:
    autonutils.drive_to_point(19.63, 119.14, 46.50, false, false, NULL, 0, 2000);
    stop_drive_motors();

    //reset the number of balls counted:
    lower_balls_counted = 0;
    upper_balls_counted = 0;

    // score in tower three:
    score_in_goal_with_light(1);
    indexer = 127;
    set_intake(127);
    wait_until_number_of_lower_balls_counted(2);
    set_intake(0);

    //dispense the blue balls from tower two and three:
    autonutils.drive_to_point(-2.96, 92.82, 46.11, false, true, []() { flywheel = -127; indexer = -127; set_intake(-127); }, 32);
    autonutils.drive_to_point(-12.14, 97.69, 243.98, false, false, NULL, 0, 2000);
    flywheel = 0;
    indexer = 127;
    set_intake(127);

    //collect the next ball and drive to tower four:
    autonutils.drive_to_point(-23.64, 93.56, 242.01, false, false);
    autonutils.drive_to_point(-36.49, 113.07, 356.42, false, false, NULL, 0, 2000);
    stop_drive_motors();

    //reset the number of balls counted:
    lower_balls_counted = 0;
    upper_balls_counted = 0;

    //score in tower four
    score_in_goal_with_light(1);
    indexer = 127;
    // set_intake(127);
    // wait_until_number_of_lower_balls_counted(1);
    set_intake(0);
    
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{
    pros::Task autonomous_debugger(debug_autonomous);
    switch (auton_mode)
    {

    case auton_modes::home_row:
        run_homerow();
        break;
    case auton_modes::skills:
        //put code for a skills run
        run_skills();
        break;
    }
}
