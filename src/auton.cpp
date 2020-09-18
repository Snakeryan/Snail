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
}

void run_field_sides()
{
    drivetrain.set_current_global_position(0, 0, 0);

    scorer.set_intakes(127);
    scorer.set_flywheel(-127);
    scorer.set_indexers(127);

    // =========== WAYPOINT TO BALL C ===========
    drivetrain.drive_to_point(-0.08, -24.61, 314.92, 2, 1);

    // =========== COLLECT BALL C ===========
    drivetrain.drive_to_point(-12.43, -14.16, 315.90, 2, 3);

    // =========== COLLECT BALL D ===========
    drivetrain.drive_to_point(-20.27, -35.57, 225.24, 2, 3);

    // =========== DRIVE TO TOWER THREE ===========
    drivetrain.drive_to_point(-33.14, -44.95, 319.14, 2, 1);
    // drivetrain.drive_to_point(-36.57, -41.71, 319.24, 2, 3, NULL, 0, 1500);
    drivetrain.stop_drive_motors();

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(319.24, false, 1500);
    pros::delay(200);
    drivetrain.set_current_global_position(0, 0, 0);

    // =========== BACKUP FROM TOWER THREE ===========
    drivetrain.drive_to_point(-0.70, -7.97, 1.77, 2, 1);

    // =========== WAYPOINT TO BALL E ===========
    drivetrain.drive_to_point(-33.57, -1.98, 3.34, 2, 3);

    // =========== COLLECT BALL E ===========
    drivetrain.drive_to_point(-33.70, 5.17, 2.95, 2, 3);

    // =========== DRIVE TO TOWER FOUR ===========
    drivetrain.drive_to_point(-37.68, 5.32, 331.42, 1, 1);
    drivetrain.drive_to_point(-47.49, 5.96, 320.52, 1, 1);
    drivetrain.drive_to_point(-52.83, 8.43, 323.76, 2, 1, NULL, 0, 2000);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(317, true, 1500);

    // pros::delay(200);
    // drivetrain.set_current_global_position(0, 0, 0);

    // // =========== COLLECT BALL E ===========
    // drivetrain.drive_to_point(4.66, -9.71, 336.23, 2, 1);
    // drivetrain.drive_to_point(12.51, -9.38, 248.22, 2, 3);
}

void run_skills()
{
    run_field_sides();
    drivetrain.stop_drive_motors();
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
