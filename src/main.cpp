#include "main.h"
#include "Scorer.h"
#include "auton.h"
#include "Drivetrain.h"
#include "globals.h"

void display_data()
{
	while (true)
	{
		pros::lcd::set_text(1, "(X, Y): (" + std::to_string(drivetrain.get_globalX()) + ", " + std::to_string(drivetrain.get_globalY()) + ")");
		pros::lcd::set_text(2, "alpha: " + std::to_string(drivetrain.get_alpha_in_degrees()));
		pros::lcd::set_text(4, std::to_string((int)indexer.get_temperature()) + "; " + std::to_string((int)flywheel.get_temperature()));
		pros::lcd::set_text(3, std::to_string((int)FL.get_temperature()) + "; " + std::to_string((int)FR.get_temperature()) + "; " + std::to_string((int)BL.get_temperature()) + "; " + std::to_string((int)BR.get_temperature()));
		pros::lcd::set_text(2, "alpha with imu: " + std::to_string(IMU.get_heading()));
		pros::Task::delay(20);
	}
}

void display_auton_mode()
{
	switch (auton_mode)
	{
	case auton_modes::home_row:
		pros::lcd::set_text(0, "home row mode");
		break;
	case auton_modes::skills:
		pros::lcd::set_text(0, "skills mode");
		break;
	default:
		pros::lcd::set_text(0, "no mode");
		break;
	}
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button()
{
	auton_mode = static_cast<auton_modes::Auton_mode>(static_cast<int>(auton_mode) + 1);
	if (auton_mode > auton_modes::skills)
	{
		auton_mode = auton_modes::home_row;
	}
	display_auton_mode();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	scorer.setup();
	drivetrain.setup();
	pros::Task display_data_task(display_data);
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
	display_auton_mode();
}

void stop_all_motors()
{
	drivetrain.stop_drive_motors();
	scorer.stop_motors();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	// stop_all_motors();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

//Helper functions:

//driver control functions:

//Macros:
/*
launch (L1)
collect (L2) (intake and indexers at half speed)
dispence (R1)
lower indexers down (R2)
*/

void run_macros()
{
	const int light_sensor_threshold = -500;
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		scorer.set_intake(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		scorer.set_intake(-63);
	}
	else
	{
		scorer.set_intake(0);
	}

	// If both buttons are pressed and limit switch is detected: stop indexer
	// If only B is pressed: reverse indexer
	// If only L2 is pressed: indexer
	// Else: stop indexer

	if ((scorer.get_light_calibrated_value() < light_sensor_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		// indexer
		scorer.set_indexer(0);
		pros::delay(50);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		scorer.set_indexer(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
	{
		scorer.set_indexer(-127);
	}

	else
	{
		scorer.set_indexer(0);
	}

	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		// flywheel negative, manual dispensing
		scorer.set_flywheel(-127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		//flywheel positive, shooting
		scorer.set_flywheel(127);
	}
	else
	{
		scorer.set_flywheel(0);
	}
}

void opcontrol()
{
	while (true)
	{
		drivetrain.driver_control(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
								  controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
								  controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		run_macros();
		pros::delay(20);
	}
}
