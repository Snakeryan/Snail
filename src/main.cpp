#include "main.h"
#include "auton.h"
#include "drivetrain.h"
#include "globals.h"

void calibrate_IMU()
{
	pros::lcd::set_text(5, "Calibrating IMU");
	IMU.reset();

	while (IMU.is_calibrating())
	{
		pros::delay(10);
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

void setup_sensors()
{
	calibrate_IMU();
	vision_sensor.set_signature(1, &BLUE_BALL_SIGNATURE);
	vision_sensor.set_signature(2, &RED_BALL_SIGNATURE);
	vision_sensor.set_signature(3, &tower_backboard_signature);

	pros::Task auton_sensors_task(run_auton_sensors);
}

void manage_indexer_and_flywheel()
{
	pros::vision_object_s_t rtn;
	while (true)
	{
		rtn = vision_sensor.get_by_size(0);
		if (auto_sort_balls)
		{
			if (rtn.signature == 1) //blue
			{
				flywheel = -127;
				pros::delay(100);
			}
			else if (rtn.signature == 2) // red
			{
				flywheel = 127;
			}
			else
			{
				flywheel = 127;
			}
		}
		if (dispense_triggered)
		{
			indexer = -127;
			wait_until_number_of_lower_balls_counted(lower_balls_counted + 1);
			indexer = 127;
			flywheel = -127;
			dispense_triggered = false;
		}

		pros::Task::delay(20);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	light_sensor.calibrate();
	setup_sensors();
	drivetrain.make_odometry_update_thread();
	pros::Task flywheel_manager_task(manage_indexer_and_flywheel);
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
	display_auton_mode();
}

void stop_all_motors()
{
	FL = 0;
	FR = 0;
	BL = 0;
	BR = 0;

	flywheel = 0;
	indexer = 0;

	set_intake(0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
	stop_all_motors();
	light_sensor.calibrate();
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

void drive()
{
	const int deadzone = 10;

	int Xaxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	int Yaxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

	int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

	FL = Yaxis + Xaxis + (turn);
	FR = -Yaxis + Xaxis + (turn);
	BR = -Yaxis - Xaxis + (turn);
	BL = Yaxis - Xaxis + (turn);
}

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
		set_intake(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		set_intake(-63);
	}
	else
	{
		set_intake(0);
	}

	// If both buttons are pressed and limit switch is detected: stop indexer
	// If only B is pressed: reverse indexer
	// If only L2 is pressed: indexer
	// Else: stop indexer

	// if ((get_light_calibrated_value() < light_sensor_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	// {
	// 	// indexer
	// 	indexer = 0;
	// 	pros::delay(50);
	// }
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		indexer = 127;
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
	{
		indexer = -127;
	}

	else
	{
		indexer = 0;
	}

	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		// flywheel negative, manual dispensing
		flywheel = -127;
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		//flywheel positive, shooting
		flywheel = 127;
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		auto_sort_balls = true;
	}
	else
	{
		auto_sort_balls = false;
		flywheel = 0;
	}
}

void display_data()
{
	while (true)
	{
		pros::lcd::set_text(1, "(X, Y): (" + std::to_string(drivetrain.get_globalX()) + ", " + std::to_string(drivetrain.get_globalY()) + ")");
		pros::lcd::set_text(2, "alpha: " + std::to_string(drivetrain.get_alpha_in_degrees()));
		pros::lcd::set_text(4, std::to_string((int)indexer.get_temperature()) + "; " + std::to_string((int)flywheel.get_temperature()));
		// pros::lcd::set_text(3, std::to_string((int)FL.get_temperature()) + "; " + std::to_string((int)FR.get_temperature()) + "; " + std::to_string((int)BL.get_temperature()) + "; " + std::to_string((int)BR.get_temperature()));
		pros::lcd::set_text(2, "alpha with imu: " + std::to_string(IMU.get_heading()));
		pros::Task::delay(20);
	}
}

void opcontrol()
{
	pros::Task display_data_task(display_data);
	while (true)
	{
		drive();
		run_macros();
		pros::delay(20);
	}
}
