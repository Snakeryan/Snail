#include "main.h"
#include "auton.h"
#include "Drivetrain.h"
#include "globals.h"
#include "PID_controller.h"
#include "pros/rtos.hpp"

// NEED TO CREATE A TASK THAT KEEPS TRACK OF THE BALLS COLLECTED AND SCORES THAT AMOUNT OF BALLS WHEN A CERTAIN CENTER_ON_TOWER_BACKBOARD_WITH_BUMPER IS ACTIVATED
// THIS FUNCTION COULD ALSO BE TOLD WHEN TO BE ACTIVATED (COORDINATE AWAY FROM BALL) AND DEACTIVATED (COORDINATE AFTER BALL) THIS WILL MAKE SURE THE SYSTEM DOES NOT COUNT BLUE BALLS OR OVER COUNT
double stop_watch;
bool is_start_stop_watch = false;

double get_stop_watch()
{
    return stop_watch;
}
void start_stop_watch()
{
    double time_before_init, stop_watch_time;
    while (true)
    {
        if (is_start_stop_watch)
        {
            stop_watch_time = pros::millis() - time_before_init;
            stop_watch = stop_watch_time / 1000.0;
        }
        else
        {
            time_before_init = pros::millis() - stop_watch_time;
        }
        pros::delay(10);
    }
}

void run_homerow()
{
}

void test_mode()
{
    scorer.reset_balls_counted();
    scorer.score_n_balls(1);
    drivetrain.stop_drive_motors();
}


void run_skills_start()
{

}

void run_field_sides(bool is_second_call)
{
 
}

void run_blue_front() {
 
}


void run_skills_end()
{
}

void run_skills()
{

    if (true)
    {
        test_mode();
        drivetrain.stop_drive_motors();
        scorer.set_intakes(0);
        scorer.set_flywheel(0);
        scorer.set_indexers(0);
        drivetrain.stop_drive_motors();
        return;
    }

    is_start_stop_watch = true;

    drivetrain.reset_odom();
    // push
    // push
    // push
    // push
    // push
    // push
    // push
    // push
    // push
    
        
    

    // run_skills_start();

    run_field_sides(false);

    // run_blue_front();

    // run_field_sides(true);

    // run_skills_end();

    is_start_stop_watch = false;
    scorer.set_indexers(0);

    pros::lcd::set_text(6, "time taken: " + std::to_string(get_stop_watch()));

        // push
    // push
    // push
    // push
    // push
    // push
    // push
    // push
    // push
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
    pros::lcd::initialize();

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
