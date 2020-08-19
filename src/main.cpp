#include "main.h"
#include "auton.h"
#include "auton_utils.h"
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
	light_sensor.calibrate();
	calibrate_IMU();

	vision_sensor.set_signature(1, &BLUE_BALL_SIGNATURE);
	vision_sensor.set_signature(2, &RED_BALL_SIGNATURE);

	pros::Task auton_sensors_task(run_auton_sensors);
}

void manage_flywheel()
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
				pros::delay(200);
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
		pros::lcd::set_text(7, "signature: " + std::to_string(rtn.signature));
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
	setup_sensors();
	autonutils.make_update_thread();
	pros::Task flywheel_manager_task(manage_flywheel);
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
	int ball_brightness_threshold = 2000;

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

	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && (!(light_sensor.get_value() < ball_brightness_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))))
	{
		// indexer
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
		pros::lcd::set_text(1, "(X, Y): (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
		pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
		pros::lcd::set_text(3, "light: " + std::to_string(light_sensor.get_value()));
		pros::lcd::set_text(4, std::to_string((int)indexer.get_temperature()) + "; " + std::to_string((int)flywheel.get_temperature()));
		pros::lcd::set_text(5, std::to_string((int)FL.get_temperature()) + "; " + std::to_string((int)FR.get_temperature()) + "; " + std::to_string((int)BL.get_temperature()) + "; " + std::to_string((int)BR.get_temperature()));
		pros::Task::delay(20);
	}
}

void opcontrol()
{
	pros::Task balls_counted(display_data);
	while (true)
	{
		drive();
		run_macros();
		pros::delay(20);
	}
}
