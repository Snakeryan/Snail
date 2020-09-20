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

    // drivetrain.use_IMU_for_odometry(false);
    // drivetrain.set_current_global_position(0, 0, 0);
    // drivetrain.drive_to_point(30, 0, 0);
    // drivetrain.drive_to_point(0, 0, 0);
    // drivetrain.stop_drive_motors();

    // drivetrain.use_IMU_for_odometry(false);
    // drivetrain.set_current_global_position(0, 0, 0);
    // drivetrain.drive_to_point(0, 30, 180);
    // drivetrain.drive_to_point(0, 0, 0);
    // drivetrain.stop_drive_motors();
    //     drivetrain.use_IMU_for_odometry(true);
    //     drivetrain.set_current_global_position(0, 0, 0);
    //     drivetrain.drive_to_point(30, 0, 0);
    //     drivetrain.drive_to_point(0, 0, 0);
    //     drivetrain.stop_drive_motors();
    // drivetrain.use_IMU_for_odometry(true);
    // drivetrain.set_current_global_position(0, 0, 0);
    // drivetrain.drive_to_point(0, 30, 180);
    // drivetrain.drive_to_point(0, 0, 0);
    // drivetrain.stop_drive_motors();
    // }
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
    drivetrain.drive_to_point(-19.80, -30.51, 227.21, 2, 3);
    drivetrain.drive_to_point(-33.01, -44.27, 226.81, 2, 3);

    drivetrain.point_turn_PID(312, false);
    // =========== DRIVE TO TOWER THREE ===========
    drivetrain.drive_to_point(-36.51, -40.24, 311.58, 2, 1, NULL, 0, 1500);
    drivetrain.stop_drive_motors();

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(312, false, 500);
    pros::delay(200);
    drivetrain.set_current_global_position(0, 0, 0);

    // =========== BACKUP FROM TOWER THREE ===========
    drivetrain.drive_to_point(-0.70, -7.97, 1.77, 2, 1);

    // =========== WAYPOINT TO BALL E ===========
    drivetrain.drive_to_point(-33.57, -1.98, 3.34, 2, 3);

    // =========== COLLECT BALL E ===========
    drivetrain.drive_to_point(-33.70, 5.17, 2.95, 2, 3);

    // =========== DRIVE TO TOWER FOUR ===========
    drivetrain.drive_to_point(-44.73, 5.74, 339.86, 1, 1);
    drivetrain.drive_to_point(-53.46, 2.36, 321.79, 2, 1);

    drivetrain.stop_drive_motors();

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(319, true, 1500);
    pros::delay(200);
    drivetrain.set_current_global_position(0, 0, 0);
}

void run_skills()
{
    // if (true)
    // {
    //     test_mode();
    //     drivetrain.stop_drive_motors();
    // //     return;
    // // }

    run_field_sides();
    drivetrain.set_current_global_position(0, 0, 0);

    scorer.set_intakes(127);
    scorer.set_flywheel(-127);
    scorer.set_indexers(127);

    // =========== BACK OUT OF TOWER FOUR ===========
    drivetrain.drive_to_point(6.75, -8.09, 339.96, 1, 3);

    // =========== WAYPOINT TO BALL F ===========
    drivetrain.drive_to_point(11.52, -33.94, 314, 2, 1, NULL, 0, 1500);

    // =========== COLLECT BALL F ===========

    drivetrain.drive_to_point(-1.11, -25.61, 311.77, 2, 1);

    // =========== WAYPOINT TO TOWER FIVE ===========
    drivetrain.drive_to_point(-27.59, -47.02, 313.64, 2, 3);

    // =========== DRIVE TO TOWER FIVE ===========
    drivetrain.drive_to_point(-31.42, -47.05, 313.34, 2, 3);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(313, true, 1000);
    pros::delay(200);
    drivetrain.reset_odom();
    drivetrain.stop_drive_motors();
    // return;

    // =========== BACK OUT OF TOWER FIVE ===========
    drivetrain.drive_to_point(-10.79, -20.79, 307.16, 2, 1);

    // =========== COLLECT BALL G ===========
    drivetrain.drive_to_point(-21.85, -15.82, 313.25, 2, 1, NULL, 0, 2000);

    // =========== DRIVE TO TOWER SIX ===========
    drivetrain.drive_to_point(-53.06, 1.92, 315.90, 2, 1, NULL, 0, 2000);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(316, true, 1000);
    drivetrain.set_current_global_position(0, 0, 0);
    drivetrain.stop_drive_motors();

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
