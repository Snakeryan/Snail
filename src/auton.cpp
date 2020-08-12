#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "helper.h"

bool sort_balls = false;
int balls_counted = 0;

AutonUtils autonutils(1.375, 6.86024, 6.86024, 6.86024, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM);

void score(int flywheel_power, int indexer_power)
{
    flywheel = flywheel_power;
    indexer = indexer_power;
}

void auton_color_sorting()
{
    pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
    pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);

    vision_sensor.set_signature(1, &BLUE_BALL_SIGNATURE);
    vision_sensor.set_signature(2, &RED_BALL_SIGNATURE);

    while (true)
    {
        if (sort_balls)
        {
            pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
            // Gets the largest object of the EXAMPLE_SIG signature
            pros::lcd::set_text(2, "signature: " + std::to_string(rtn.signature));
            pros::lcd::set_text(3, "w: " + std::to_string(rtn.width));
            pros::lcd::set_text(4, "h: " + std::to_string(rtn.height));
            pros::delay(10);

            if (rtn.signature == 2) //red
            {
                flywheel = -127/2;
                pros::delay(200);
            }
            else if (rtn.signature == 1) // blue
            {
                flywheel = 127;
            }
            else
            {
                flywheel = 127;
            }
        }
        pros::Task::delay(10);
    }
}

void debug_autonomous()
{
    while (true)
    {
        pros::lcd::set_text(1, "coordinates: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
        pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
        pros::Task::delay(10);
    }
}

void ball_counter()
{
    int prev_limit_value = 0;
    while (true)
    {
        if (limit_switch.get_value() == 1 && prev_limit_value == 0)
        {
            balls_counted++;
        }
        // pros::lcd::set_text(6, "the limit switch value is: " + std::to_string(limit_value));
        prev_limit_value = limit_switch.get_value();
        pros::Task::delay(20);
    }
}
void run_odometry_mode()
{
    autonutils.make_update_thread();
    while(true)
    {
        // autonutils.update();

        pros::lcd::set_text(1, "coordinates: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
        pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
        pros::delay(10);
    } 
}

void stop()
{
    FL = 0;
    FR = 0;
    BL = 0;
    BR = 0;
}

void run_homerow()
{
    autonutils.make_update_thread();
    pros::Task color_sorter(auton_color_sorting);
    pros::Task autonomous_debugger(debug_autonomous);
    pros::Task ball_counter_task(ball_counter);

    pros::lcd::set_text(0, "Autonomous Mode ");
    autonutils.set_current_global_position(0, 0, 0);

    setIntake(127);
    autonutils.drive_to_point(0, -15, 45, false, true);

    // First Goal
    autonutils.drive_to_point(16.2, -6.7, 46, false, false);
    sort_balls = true;
    stop();

    score(127, 127);
    while (balls_counted < 3)
    {
        pros::delay(10);
    }
    setIntake(-31);
    pros::delay(500);

    // Waypoint to second goal
    // sort_balls = false;
    score(0, 0);
    setIntake(0);
    autonutils.drive_to_point(-37.1, -23.7, 0, false, true);
    autonutils.drive_to_point(-37.5, -9.6, 0, false, false);
    stop();
}

void run_skills()
{
}