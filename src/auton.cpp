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
    drivetrain.set_current_global_position(0, 0, 0);
    drivetrain.turn_to_point(10, 0);
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
    pros::delay(200);
    drivetrain.set_current_global_position(0, 0, 0);
}

void run_skills()
{
    if (false)
    {
        test_mode();
        drivetrain.stop_drive_motors();
    }

    // run_field_sides();

    drivetrain.set_current_global_position(0, 0, 0);

    scorer.set_intakes(127);
    scorer.set_flywheel(-127);
    scorer.set_indexers(127);

    // =========== BACK OUT OF TOWER FOUR ===========
    drivetrain.drive_to_point(2.10, -5.68, 344.58, 1, 1);

    // =========== WAYPOINT TO BALL F ===========
    drivetrain.drive_to_point(11.69, -9.97, 249.30, 2, 0, NULL, 0, 1500);

    // =========== TURN TO BALL F ===========
    drivetrain.turn_to_point(-6.91, -19.00);
    drivetrain.set_current_global_position(drivetrain.get_globalX(), drivetrain.get_globalY(), IMU.get_heading());

    // =========== COLLECT BALL F ===========
    drivetrain.drive_to_point(-13.53, -20.26, 257.26, 2, 3);

    // =========== WAYPOINT TO TOWER FIVE ===========
    drivetrain.drive_to_point(-24.82, -34.98, 275.82, 1, 1);

    // =========== DRIVE TO TOWER FIVE ===========
    drivetrain.drive_to_point(-30.27, -35.54, 274.12, 2, 3, NULL, 0, 3000);
    drivetrain.stop_drive_motors();
    drivetrain.set_current_global_position(drivetrain.get_globalX(), drivetrain.get_globalY(), IMU.get_heading());
    pros::delay(1000);

    // =========== BACK OUT OF TOWER FIVE ===========
    drivetrain.drive_to_point(-20.67, -38.01, 225.83, 1, 1);

    // =========== COLLECT BALL G ===========
    drivetrain.drive_to_point(-43.52, -59.36, 226.15, 2, 3);
    drivetrain.drive_to_point(-56.68, -71.52, 235.29, 2, 3);

    // =========== DRIVE TO TOWER SIX ===========
    drivetrain.turn_to_point(-68.14, -81.02);
    drivetrain.drive_to_point(-68.14, -81.02, 266.49, 2, 3, NULL, 0, 1500);
     drivetrain.drive_to_point(-74.74, -80.10, 265.09, 2, 3, NULL, 0, 2000);
    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(268, true, 2000);
    pros::delay(200);
    // drivetrain.set_current_global_position(0, 0, 0);

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
