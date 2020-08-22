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

void run_auton_sensors()
{
    int prev_lower_limit_value = 0;
    int prev_upper_limit_value = 0;
    double prev_time = pros::millis();
    while (true)
    {

        if (lower_limit_switch.get_value() == 1 && prev_lower_limit_value == 0)
        {
            lower_balls_counted++;
        }

        prev_lower_limit_value = lower_limit_switch.get_value();

        if ((upper_limit_switch.get_value() == 1 && prev_upper_limit_value == 0) && abs(prev_time - pros::millis()) > 40)
        {
            upper_balls_counted++;
            prev_time = pros::millis();
        }

        prev_upper_limit_value = upper_limit_switch.get_value();

        pros::lcd::set_text(3, "left: " + std::to_string(autonutils.get_left_encoder_distance()));
        pros::lcd::set_text(4, "right: " + std::to_string(autonutils.get_right_encoder_distance()));
        pros::lcd::set_text(6, "lower limit: " + std::to_string(lower_balls_counted));
        pros::lcd::set_text(7, "upper limit: " + std::to_string(upper_balls_counted));

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
    while (upper_balls_counted <= num_balls)
    {
        flywheel = 127;
        if (upper_limit_switch.get_value() == 0)
        {
            indexer = 127;
        }
        else
        {
            indexer = 0;
            pros::delay(50);
        }
    }
    pros::delay(50);
    flywheel = -63;
    pros::delay(100);
    flywheel = 0;
}

void run_skills()
{
    autonutils.set_current_global_position(0, 0, 0);
    set_intake(127);
    autonutils.drive_to_point(0, 13.72, 0, false, true);
    autonutils.drive_to_point(20.23, 7.25, 135, false, false);
    stop_drive_motors();

    // Score 2 red balls

    score_in_goal(2);
    // Store the 2 blue balls

    flywheel = 0;
    indexer = 0;
    set_intake(0);
    // wait_until_number_of_lower_balls_counted(2);
    // set_intake(-63);
    // pros::delay(50);
    // set_intake(0);
    // autonutils.drive_to_point(7.54, 29.62, 326, false, true);
    // set_intake(127);
    // // dispence the two blue balls
    // flywheel = -127;
    // autonutils.drive_to_point(-5.88, 50.26, 330, false, false);
    // flywheel = 0;

    // // Move to the next point
    // stop_drive_motors();
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
