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
    // scorer.set_intakes(127);
    // scorer.set_indexers(127);
    // scorer.set_flywheel(127);
    drivetrain.drive_to_point(0, 10, 0, false, false, NULL, 0, 1000);
    drivetrain.drive_to_tower_backboard(0, true);
    drivetrain.stop_drive_motors();

    // pros::lcd::set_text(5, "average error: " + std::to_string(pid_controller.get_error_average(5)));
}

void run_skills()
{

    drivetrain.set_current_global_position(0, 0, 0);
    if (true)
    {
        test_mode();
        return;
    }
    scorer.set_intakes(127);
    drivetrain.drive_to_point(0, 13.72, 0, false, true);
    drivetrain.drive_to_point(20.10, 6.70, 135.5, false, false, NULL, 0, 3000);
    drivetrain.stop_drive_motors();

    scorer.reset_balls_counted();

    // Score 2 red balls and Store 2 blue balls:

    // scorer.score_n_balls(2);
    // scorer.collect_n_balls(2);
    // scorer.wait_until_number_of_lower_balls_counted(2);
    // scorer.wait_until_number_of_uppper_balls_counted(2);

    //WAYPOINT to tower two:
    drivetrain.drive_to_point(
        7.54, 29.62, 200, false, true); //, []() { scorer.dispense(); }, 15);
    // dispense the two blue balls and collect a red ball:

    scorer.set_intakes(127);
    scorer.set_indexers(127);
    scorer.set_flywheel(-127);

    // drivetrain.drive_to_point(
    //     -2.15, 49.93, 329.7, false, false, []() { scorer.set_indexers(127); scorer.set_flywheel(0); }, 3);

    drivetrain.drive_to_point(
        -6.72, 40.17, 346.43, false, false); //,[]() { scorer.set_indexers(127); scorer.set_flywheel(0); }, 3);

    drivetrain.drive_to_point(-4.69, 49.64, 338.82, false, false);
    drivetrain.drive_to_point(-12.14, 62.60, 359.41, false, false);
    drivetrain.stop_drive_motors();

    //drive and turn to tower two:
    drivetrain.point_turn_PID(91.34);
    // scorer.set_indexers(127);
    drivetrain.drive_to_point(
        20, 60.97, 91.34, false, false, NULL, 0, 2000);
    // scorer.set_flywheel(0);
    drivetrain.drive_to_tower_backboard(90);
    pros::delay(200);
    drivetrain.set_current_global_position(16.077751092179696, 63.107507307056174, drivetrain.get_alpha_in_degrees());

    //reset the number of balls counted:
    scorer.reset_balls_counted();

    //score and collect in tower two:
    // scorer.score_n_balls(2);
    // scorer.collect_n_balls(1);

    // WAYPOINT to tower three:
    drivetrain.drive_to_point(-0.33655291345362054, 104.75536482291518, 38.44431091681539, false, true);

    //drive to tower three:
    drivetrain.drive_to_point(20.93, 118.60, 42.53, false, false, NULL, 0, 2000);
    drivetrain.stop_drive_motors();
    scorer.reset_balls_counted();

    // score in tower three:
    // scorer.set_intakes(127);
    // // scorer.score_in_goal_with_light(1);
    // scorer.set_indexers(127);
    // scorer.dispense();

    // scorer.wait_until_number_of_lower_balls_counted(2);
    // scorer.set_intakes(0);

    //dispense the blue balls from tower two and three:
    drivetrain.drive_to_point(
        0.23109562836991665, 103.91717864628831, 36.477850000354252, false, true); /*[]() { scorer.set_flywheel(-127);
        scorer.set_indexers(-127);
        scorer.set_intakes(-127); }, 32);*/
    drivetrain.drive_to_point(-15.90, 94.28, 245.61, false, false, NULL, 0, 2000);
    scorer.set_flywheel(0);
    // scorer.set_indexers(127);
    // scorer.set_intakes(127);
    scorer.dispense();

    //collect the next ball and drive to tower four:
    drivetrain.drive_to_point(-22.00, 90.62, 250.04, false, false);
    drivetrain.drive_to_point(-35.18, 81.66, 264.49, false, true);
    drivetrain.drive_to_point(-36.98, 112.80, 1.24, false, false, NULL, 0, 3000);
    drivetrain.stop_drive_motors();


    scorer.reset_balls_counted();

    //score in tower four
    // scorer.score_in_goal_with_light(1);
    // scorer.set_indexers(127);
    // set_intake(127);
    // wait_until_number_of_lower_balls_counted(1);
    scorer.set_intakes(0);
    drivetrain.drive_to_tower_backboard(0, true);
    pros::delay(200);
    drivetrain.set_current_global_position(0, 0, IMU.get_heading());
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
