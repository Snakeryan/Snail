#include "main.h"
#include "auton.h"
#include "Drivetrain.h"
#include "globals.h"
#include "PID_controller.h"

void run_homerow()
{
}

void test_mode()
{
    drivetrain.drive_to_point(0, 10, 0, false, false, NULL, 0, 1000);
    drivetrain.drive_to_tower_backboard(0);
    drivetrain.stop_drive_motors();
    // pros::lcd::set_text(5, "average error: " + std::to_string(pid_controller.get_error_average(5)));
}

void run_skills()
{

    drivetrain.set_current_global_position(0, 0, 0);
    test_mode();
    return;
    scorer.set_intakes(127);
    drivetrain.drive_to_point(0, 13.72, 0, false, true);
    drivetrain.drive_to_point(20.10, 6.70, 135.5, false, false, NULL, 0, 3000);
    drivetrain.stop_drive_motors();

    scorer.reset_balls_counted();

    // Score 2 red balls and Store 2 blue balls:

    scorer.score_n_balls(2);
    scorer.collect_n_balls(2);
    scorer.wait_until_number_of_lower_balls_counted(2);
    scorer.wait_until_number_of_uppper_balls_counted(2);

    //WAYPOINT to tower two:
    drivetrain.drive_to_point(
        7.54, 29.62, 200, false, true, []() { scorer.dispense(); }, 15);

    // dispense the two blue balls and collect a red ball:

    scorer.set_intakes(127);

    // drivetrain.drive_to_point(
    //     -2.15, 49.93, 329.7, false, false, []() { scorer.set_indexers(127); scorer.set_flywheel(0); }, 3);

    drivetrain.drive_to_point(
        -6.43, 50.2, 327.7, false, false, []() { scorer.set_indexers(127); scorer.set_flywheel(0); }, 3);

    drivetrain.drive_to_point(-11.01, 62.10, 0, false, false);

    //drive and turn to tower two:
    drivetrain.point_turn_PID(91.34);
    scorer.set_indexers(127);
    drivetrain.drive_to_point(
        25.81, 60.97, 91.34, false, false, NULL, 0, 2500);
    scorer.set_flywheel(0);
    drivetrain.drive_to_tower_backboard(90);
    drivetrain.set_current_global_position(0, 0, 0);
    drivetrain.stop_drive_motors();

    //reset the number of balls counted:
    scorer.reset_balls_counted();

    //score and collect in tower two:
    scorer.score_n_balls(2);
    scorer.collect_n_balls(1);

    //WAYPOINT to tower three:
    // drivetrain.drive_to_point(-1.40, 110.37, 52, false, true);

    return;

    //drive to tower three:
    drivetrain.drive_to_point(19.63, 119.14, 46.50, false, false, NULL, 0, 2000);
    drivetrain.stop_drive_motors();

    scorer.reset_balls_counted();

    // score in tower three:
    scorer.set_intakes(127);
    // scorer.score_in_goal_with_light(1);
    scorer.set_indexers(127);

    scorer.wait_until_number_of_lower_balls_counted(2);
    scorer.set_intakes(0);

    //dispense the blue balls from tower two and three:
    drivetrain.drive_to_point(
        -2.96, 92.82, 46.11, false, true, []() { scorer.set_flywheel(-127);
        scorer.set_indexers(-127);
        scorer.set_intakes(-127); }, 32);
    drivetrain.drive_to_point(-12.14, 97.69, 243.98, false, false, NULL, 0, 2000);
    scorer.set_flywheel(0);
    scorer.set_indexers(127);
    scorer.set_intakes(127);

    //collect the next ball and drive to tower four:
    drivetrain.drive_to_point(-23.64, 93.56, 242.01, false, false);
    drivetrain.drive_to_point(-36.49, 113.07, 356.42, false, false, NULL, 0, 2000);
    drivetrain.stop_drive_motors();

    scorer.reset_balls_counted();

    //score in tower four
    // scorer.score_in_goal_with_light(1);
    scorer.set_indexers(127);
    // set_intake(127);
    // wait_until_number_of_lower_balls_counted(1);
    scorer.set_intakes(0);
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
