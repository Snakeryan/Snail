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
    drivetrain.center_on_tower_with_bumper(0, false);
}
void run_field_sides()
{
    drivetrain.reset_odom();

    scorer.reset_balls_counted(); //DELETE LATER

    scorer.score_n_balls(1);

    // scorer.collect_n_balls(2);

    // =========== FINISH SCORING AND COLLECTING IN TOWER TWO/SIX ===========
    scorer.wait_until_number_of_uppper_balls_counted(1);

    // scorer.wait_until_number_of_lower_balls_counted(2);

    // // =========== DISPENSE TWO BALLS ===========
    // scorer.reset_balls_counted();
    // scorer.dispense_n_balls(2, true);

    // =========== WAYPOINT TO BALL C/H ===========
    drivetrain.drive_to_point(
        -0.08, -24.61, 314.92, 1, 1);

    // =========== TURN ON INDEXERS AND INTAKES POSITIVELY ===========
    scorer.set_intakes(127);
    scorer.set_indexers(127);
    scorer.set_flywheel(0);

    // =========== COLLECT BALL C/H ===========
    drivetrain.drive_to_point(-12.43, -14.16, 315.90, 2, 3, NULL, 0, 1500);

    // =========== WAYPOINT TO BALL C/H ===========
    drivetrain.drive_to_point(-19.80, -30.51, 227.21, 2, 3, NULL, 0, 1500);

    // =========== COLLECT BALL D/I ===========
    drivetrain.drive_to_point(-33.01, -44.27, 226.81, 2, 3, NULL, 0, 1500);

    // =========== DRIVE TO TOWER THREE/SEVEN ===========
    drivetrain.drive_to_point(
        -35.18, -43.78, 313.93, 2, 3, []() { scorer.set_flywheel(-127); scorer.set_indexers(-25); }, 2, 1500);
    drivetrain.stop_drive_motors();

    // =========== SCORE AND COLLECT IN TOWER THREE/SEVEN ===========
    scorer.reset_balls_counted();

    scorer.set_intakes(0); //DELETE LATER

    scorer.score_n_balls(2);

    // scorer.collect_n_balls(1);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(312, false, 1500, false);
    pros::delay(200);
    drivetrain.reset_odom();

    // =========== FINISH SCORING AND COLLECTING IN TOWER THREE/SEVEN ===========

    // scorer.wait_until_number_of_lower_balls_counted(1);

    scorer.wait_until_number_of_uppper_balls_counted(2);

    // // =========== DISPENSE BALLS FROM TOWER THREE/SEVEN ===========
    // scorer.reset_balls_counted();
    // scorer.dispense_n_balls(1);
    // scorer.wait_until_number_of_balls_dispensed(1);

    // =========== BACK OUT OF TOWER THREE/SEVEN ===========
    drivetrain.drive_to_point(-0.70, -7.97, 1.77, 2, 1);

    // =========== WAYPOINT TO BALL E/J ===========
    drivetrain.drive_to_point(-32.93, -11.00, 356.46, 2, 3);

    // =========== COLLECT BALL E/K ===========
    drivetrain.drive_to_point(
        -33.70, 5.17, 2.95, 2, 3, []() {
        scorer.set_intakes(127);
        scorer.set_indexers(127);
        scorer.set_flywheel(0); }, 30, 1000);

    // =========== DRIVE TO TOWER FOUR/EIGHT ===========
    drivetrain.drive_to_point(-43.70, 1.94, 343.70, 1, 1, NULL, 0, 1500);
    drivetrain.drive_to_point(-52.72, 2.06, 314.03, 2, 3, NULL, 0, 1500);

    drivetrain.stop_drive_motors();

    // =========== SCORE AND COLLECT IN TOWER FOUR/EIGHT ===========
    scorer.reset_balls_counted();

    scorer.set_intakes(0); // DELETE LATER

    scorer.score_n_balls(1);

    // scorer.collect_n_balls(2);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(319, true, 1500, false);
    pros::delay(200);
    drivetrain.reset_odom();

    // FINISH SCORING AND COLLECTING IN TOWER FOUR/EIGHT

    // scorer.wait_until_number_of_lower_balls_counted(2);

    scorer.wait_until_number_of_uppper_balls_counted(1);
}

void run_blue_front()
{
    // =========== BACK OUT OF TOWER FOUR ===========
    drivetrain.drive_to_point(6.75, -8.09, 339.96, 1, 3);

    // =========== WAYPOINT TO BALL F ===========
    drivetrain.drive_to_point(11.52, -33.94, 314, 2, 1, NULL, 0, 1500);

    // =========== TURN ON INDEXERS AND INTAKES ===========
    scorer.set_indexers(127);
    scorer.set_intakes(127);

    // =========== COLLECT BALL F ===========
    drivetrain.drive_to_point(0.51, -28.05, 310.69, 2, 1);

    // =========== WAYPOINT TO TOWER FIVE ===========
    drivetrain.drive_to_point(-27.59, -47.02, 313.64, 2, 3);

    // =========== DRIVE TO TOWER FIVE ===========
    drivetrain.drive_to_point(-31.42, -47.05, 313.34, 2, 3);

    // =========== SCORE AND COLLECT IN TOWER FIVE ===========
    scorer.reset_balls_counted();

    scorer.set_intakes(0); //DELETE LATER

    scorer.score_n_balls(1);

    // scorer.collect_n_balls(1);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(313, true, 1000, false);
    pros::delay(200);
    drivetrain.reset_odom();
    drivetrain.stop_drive_motors();

    // FINISH SCORING AND COLLECTING IN TOWER FIVE
    // scorer.wait_until_number_of_lower_balls_counted(1);

    scorer.wait_until_number_of_uppper_balls_counted(1);

    // =========== BACK OUT OF TOWER FIVE ===========
    drivetrain.drive_to_point(-10.79, -20.79, 307.16, 2, 1, NULL, 0, 1000);

    // =========== TURN ON INDEXERS AND INTAKES ===========
    scorer.set_indexers(127);
    scorer.set_intakes(127);

    // =========== COLLECT BALL G ===========
    drivetrain.drive_to_point(-21.85, -15.82, 313.25, 2, 1);

    // =========== DRIVE TO TOWER SIX ===========
    drivetrain.drive_to_point(-52.47, 1.93, 314.23, 2, 1, NULL, 0, 3000);

    // =========== SCORE AND COLLECT IN TOWER SIX ===========
    scorer.reset_balls_counted();

    scorer.set_intakes(0); //DELETE LATER

    scorer.score_n_balls(1);

    // scorer.collect_n_balls(2);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(270, true, 1500, false);
    drivetrain.reset_odom();
    drivetrain.stop_drive_motors();

    // FINISH SCORING AND COLLECTING IN TOWER SIX

    // scorer.wait_until_number_of_lower_balls_counted(2);
    scorer.wait_until_number_of_uppper_balls_counted(1);
}

void run_skills_end()
{
    scorer.set_intakes(127);
    scorer.set_flywheel(-127);
    scorer.set_indexers(127);
    // =========== BACK OUT OF TOWER EIGHT ===========
    drivetrain.drive_to_point(7.68, -6.76, 305.29, 1, 1);

    // =========== TURN TO BALL J ===========
    drivetrain.turn_to_point(-1.92, -10.42);

    // =========== COLLECT TO BALL J ===========
    drivetrain.drive_to_point(-1.92, -10.42, 230.35, 2, 1);

    // =========== COLLECT TO BALL K ===========
    drivetrain.drive_to_point(-12.87, -49.84, 189.00, 2, 3);

    // =========== WAYPOINT TO TOWER NINE ===========
    drivetrain.drive_to_point(-16.65, -62.75, 137.82, 1, 3);

    // =========== CENTER ON TOWER NINE ===========
    drivetrain.center_on_tower_with_bumper(134, true, 2000);
}

void run_skills_start()
{
    drivetrain.reset_odom();

    // =========== SCORE IN TOWER ONE ===========
    scorer.reset_balls_counted();
    scorer.set_flywheel(127);
    scorer.wait_until_number_of_uppper_balls_counted(1);

    // =========== BACK OUT OF TOWER ONE ===========
    drivetrain.drive_to_point(0.37, -4.58, 1.57, 1, 1);

    // =========== WAYPOINT TO BALL A ===========
    scorer.set_indexers(127);
    drivetrain.drive_to_point(
        16.74, -10.99, 222.49, 1, 3, []() { scorer.set_intakes(-127); scorer.set_indexers(0); scorer.set_flywheel(0); }, 10);

    scorer.set_intakes(127);
    scorer.set_intakes(127);

    // =========== COLLECT TO BALL A ===========
    drivetrain.drive_to_point(11.32, -17.18, 224.36, 2, 3);

    // =========== DRIVE TO TOWER TWO ==========
    drivetrain.drive_to_point(-5.78, -43.07, 242.33, 2, 3);
    drivetrain.stop_drive_motors();

    // =========== SCORE IN TOWER TWO ===========
    scorer.reset_balls_counted();
    scorer.score_n_balls(1);
    scorer.collect_n_balls(2);

    // =========== RESET GLOBAL POSITION ===========
    drivetrain.center_on_tower_with_bumper(245, true);
    drivetrain.reset_odom();
    drivetrain.stop_drive_motors();
}

void run_skills()
{
    if (false)
    {
        test_mode();
        drivetrain.stop_drive_motors();
        return;
    }

    // run_skills_start();

    // run_field_sides();
    run_blue_front();

    return;

    run_field_sides();

    run_skills_end();
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
