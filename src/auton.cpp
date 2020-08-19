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
    int prev_limit_value = 0;
    int delta_light_sensor_value;
    int prev_light_value = light_sensor.get_value();
    while (true)
    {

        if (limit_switch.get_value() == 1 && prev_limit_value == 0)
        {
            lower_balls_counted++;
        }

        prev_limit_value = limit_switch.get_value();

        int light_sensor_value = light_sensor.get_value();
        delta_light_sensor_value = prev_light_value - light_sensor_value;
        if (delta_light_sensor_value < -80)
        {
            upper_balls_counted++;
        }

        prev_light_value = light_sensor_value;

        pros::lcd::set_text(3, "lower counter: " + std::to_string(lower_balls_counted));
        pros::lcd::set_text(4, "upper counter: " + std::to_string(upper_balls_counted));
        pros::lcd::set_text(6, "limit: " + std::to_string(lower_balls_counted));

        pros::Task::delay(10);
    }
}

void debug_autonomous()
{
    while (true)
    {
        pros::lcd::set_text(1, "x, y: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
        pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
        pros::Task::delay(10);
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
    pros::Task autonomous_debugger(debug_autonomous);

    set_intake(127);
    autonutils.drive_to_point(0, -15, 45, false, true);

    // First Goal
    autonutils.drive_to_point(16.2, -6.7, 46, false, false);
    auto_sort_balls = true;
    stop_drive_motors();

    set_flywheel_and_indexer(127, 127);
    while (lower_balls_counted < 3)
    {
        pros::delay(10);
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

void run_skills()
{
    pros::Task autonomous_debugger(debug_autonomous);

    autonutils.set_current_global_position(0, 0, 0);
    set_intake(127);
    autonutils.drive_to_point(0.96, 61.97, 270.00, false, true);
    autonutils.drive_to_point(-21.51, 62.87, 270, false, false);
    // auto_sort_balls = true;
    // set_flywheel_and_indexer(127, 127)

    go_home();
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
