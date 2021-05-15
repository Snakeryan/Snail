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

void test_mode() {
  scorer.set_indexers(127);
  scorer.set_flywheel(127);
  scorer.set_intakes(127);
  drivetrain.drive_to_point(0, 75, 0, 3, 3);
  drivetrain.drive_to_point(0, 0, 0, 3, 3);
  
    // scorer.reset_balls_counted();
    // scorer.score_n_balls(1);
    drivetrain.stop_drive_motors();
}


void run_skills_start()
{

}

void run_field_sides(bool is_second_call)
{
    // =========== TURN ON INDEXERS AND INTAKES ===========
    scorer.set_indexers(127);
    scorer.set_intakes(127);

    // =========== WAYPOINT TO BALL D ===========
    drivetrain.drive_to_point(-6.03, -20.08, 10.04, 1, 3, NULL, 0, 1450, true);

    // =========== DISPENSE BALLS FROM TOWER TWO ===========
    scorer.score_n_balls(2);

    // =========== DRIVE TO BALL D ===========
    // =========== COLLECT BALL D ===========
    drivetrain.drive_to_point(-13.27, -61.92, 348.23, 2, 3, [](){scorer.set_indexers(127); scorer.set_flywheel(0);}, 0, 2000, true);

     // =========== TURN ON INDEXERS ===========
    scorer.set_indexers(127);

    // =========== DRIVE TO BALL E ===========
    // =========== COLLECT BALL E ===========
    // =========== DRIVE TOWER THREE/SEVEN ===========
    drivetrain.drive_to_point(-36.05, -43.73, 316.39, 2, 3, [](){scorer.set_indexers(-3); scorer.set_intakes(0);}, 2, 2250, true);


    // =========== SCORE IN TOWER THREE ===========
    scorer.reset_balls_counted();
    scorer.score_n_balls(2, 1250);

    // =========== COLLECT IN TOWER THREE ===========
    scorer.set_intakes(127);
    
    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(314.93, false, 250);
    drivetrain.reset_odom();



    // =========== FINISH SCORING AND COLLECTING IN TOWER THREE/SEVEN ===========
    // scorer.wait_until_number_of_lower_balls_counted(1);
    scorer.wait_until_number_of_upper_balls_counted(2);

    // =========== START DISPENSING BALLS FROM TOWER THREE ===========

    
    // =========== BACK OUT OF TOWER THREE ===========
    drivetrain.drive_to_point(-3.63, -8.86, 342.53, 1, 1, [](){scorer.set_indexers(-127); scorer.set_flywheel(-127); scorer.set_intakes(-127);}, 9, 500, true);

    // =========== START DISPENSING BALLS FROM TOWER THREE ===========
    scorer.set_indexers(-127);
    scorer.set_flywheel(-127);
    scorer.set_intakes(-127);

    // =========== TURN ON INDEXERS AND INTAKES ===========


    // =========== COLLECT BALL F ===========
    drivetrain.drive_to_point(-33.02, 3.39, 326.63, 1, 3, [](){scorer.set_indexers(127); scorer.set_intakes(127);}, 5, 1500, true);

    // =========== DRIVE TO TOWER FOUR ===========
    drivetrain.drive_to_point(-54.89, -1.34, 310.84, 2, 3, [](){scorer.set_intakes(0);}, 0, 1500, true);

    // =========== SCORE IN TOWER FOUR ===========
    scorer.reset_balls_counted();
    scorer.score_n_balls(1, 1250);

    // =========== COLLECT IN TOWER FOUR ===========

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(306.42, false, 500);
    drivetrain.reset_odom();

    // =========== FINISH SCORING AND COLLECTING IN TOWER FOUR/EIGHT ===========
    // scorer.wait_until_number_of_lower_balls_counted(1);
    scorer.wait_until_number_of_upper_balls_counted(1);
 
}

void run_blue_front()
{
    // =========== TURN ON INDEXERS AND INTAKES ===========
    scorer.set_indexers(127);
    scorer.set_intakes(127);

    // =========== COLLECT BALL G ===========
    drivetrain.drive_to_point(-12.69, -18.63, 10.86, 2, 3, NULL, 0, 1500, true);

    // =========== WAYPOINT TO BALL H ===========
    drivetrain.drive_to_point(-13.38, -50.59, 14.35, 1, 3, NULL, 0, 1200, true);

    // =========== COLLECT BALL H ===========
    drivetrain.drive_to_point(-15.86, -61.37, 14.32, 2, 3, NULL, 0, 3000, true);

    // =========== DRIVE TO TOWER FIVE ===========
    drivetrain.drive_to_point(-37.74, -41.00, 314.78, 2, 3, NULL, 0, 3000, true);

    // =========== SCORE IN TOWER THREE ===========
    scorer.reset_balls_counted();
    scorer.score_n_balls(2, 1250);

    // =========== COLLECT IN TOWER THREE ===========
    scorer.set_intakes(127);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(312.39, false, 500);
    // drivetrain.reset_odom();

    // =========== FINISH SCORING AND COLLECTING IN TOWER THREE/SEVEN ===========
    // scorer.wait_until_number_of_lower_balls_counted(1);
    scorer.wait_until_number_of_upper_balls_counted(2);

    // // =========== START DISPENSING BALLS FROM TOWER THREE ===========
    // scorer.set_indexers(-127);
    // scorer.set_flywheel(-127);
    // scorer.set_intakes(-127);
    

 
}


void run_skills_end()
{
}

void run_skills()
{

    if (false)
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

    // run_skills_start();

    // run_field_sides(false);

    run_blue_front();

    // run_field_sides(true);

    // run_skills_end();

    is_start_stop_watch = false;
    scorer.set_indexers(0);
    drivetrain.stop_drive_motors();

    pros::lcd::set_text(1, "time taken: " + std::to_string(get_stop_watch()));

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
