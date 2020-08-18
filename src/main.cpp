#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "globals.h"
#include "helper.h"

namespace auton_modes
{
	enum Auton_mode
	{
		odometry = 1,
		home_row = 2,
		skills = 3,
	};
}

auton_modes::Auton_mode auton_mode = auton_modes::skills;

void calibrateIMU();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_left_button()
{
	// std::cout << "BUTTON PRESSED";
	// // static bool pressed = false;
	// // static int autonomous_mode = 0;
	// // pressed = !pressed;
	// // if (true)
	// // {
	// // 	autonomous_mode++;
	// // }
	// if (autonomous_mode == 0)
	// {
	// }
	// else if (autonomous_mode == 1)
	// {
	// 	pros::lcd::set_text(0, "you are in mode one of autonomous");
	// 	pros::delay(5000);
	// 	// put code for mode one:
	// 	debug_autonomous();
	// }
	// else if (autonomous_mode == 2)
	// {
	// 	pros::lcd::set_text(0, "you are in mode two of the autonomous");
	// 	pros::delay(5000);
	// 	//put code for mode two:
	// 	// run one of the autonomous programs
	// }
	// else if (autonomous_mode == 3)
	// {
	// 	pros::lcd::set_text(0, "you are in mode three of the autonomous");
	// 	pros::delay(5000);
	// 	//put code for mode three:
	// 	// run one of the autonomous programs
	// }
	// else
	// {
	// 	autonomous_mode = 0;
	// }
}

void display_auton_mode()
{
	switch (auton_mode)
	{
	case auton_modes::odometry:
		pros::lcd::set_text(0, "odometry mode");
		break;
	case auton_modes::home_row:
		pros::lcd::set_text(0, "home row mode");
		break;
	case auton_modes::skills:
		pros::lcd::set_text(0, "skills mode");
		break;
	default:
		pros::lcd::set_text(0, "no mode selected");
		break;
	}
}

void on_center_button()
{
	printf("CENTER BUTTON PRESSED \n");
	auton_mode = static_cast<auton_modes::Auton_mode>(static_cast<int>(auton_mode) + 1);
	if (auton_mode > auton_modes::skills)
	{
		auton_mode = auton_modes::odometry;
	}
	display_auton_mode();
}

void on_right_button()
{
	// printf("RIGHT BUTTON PRESSED");
	// static bool pressed = false;
	// static bool prev_pressed = false;
	// static int number_of_presses = 0;
	// pressed = !pressed;
	// if (true)
	// {
	// 	number_of_presses++;
	// }
	// if (number_of_presses == 0)
	// {
	// }
	// else if (number_of_presses == 1)
	// {
	// 	pros::lcd::set_text(0, "you are in mode one of the right button");
	// 	pros::delay(5000);
	// 	//put code for mode one:
	// }
	// else if (number_of_presses == 2)
	// {
	// 	pros::lcd::set_text(0, "you are in mode two of the right button");
	// 	pros::delay(5000);
	// 	//put code for mode two:
	// }
	// else if (number_of_presses == 3)
	// {
	// 	pros::lcd::set_text(0, "you are in mode three of the right button");
	// 	pros::delay(5000);
	// 	//put code for mode three:
	// }
	// else
	// {
	// 	number_of_presses = 0;
	// }
	// pros::lcd::set_text(0, "you are in mode: " + std::to_string(number_of_presses));
	// prev_pressed = pressed;
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
	calibrateIMU();
	autonutils.make_update_thread();
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
	display_auton_mode();
	// pros::lcd::set_text(1, "Hello World!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
	switch (auton_mode)
	{
	case auton_modes::odometry:
		run_odometry_mode();
		break;
	case auton_modes::home_row:
		run_homerow();
		break;
	case auton_modes::skills:
		//put code for a skills run
		run_skills();
		break;
	}
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

void setDrive(int F_B, int strafe, int turn)
{
	FL = F_B + strafe + (turn);
	FR = -F_B + strafe + (turn);
	BR = -F_B - strafe + (turn);
	BL = F_B - strafe + (turn);
}

//driver control functions:

void drive()
{
	const int deadzone = 10;

	int Xaxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	int Yaxis = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

	int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	// if(abs(Yaxis) < deadzone) Yaxis = 0;
	// if(abs(Xaxis) < deadzone) Xaxis = 0;

	setDrive(Yaxis, Xaxis, turn);
}
//Macros:
/*
we want to:
launch (L1)
collect (L2) (intake and indexers at half speed)
dispence (R1)
lower indexers down (R2)

*/
void run_macros()
{

	// intake 1 direction
	// indexer 1 direction
	// flywheel 2 directions (1 up and 1 do)
	long ball_brightness_threshold = 2720;
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		setIntake(127);
	}
	else
	{
		setIntake(0);
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // && !(light_sensor.get_value() < ball_brightness_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)))
	{
		// indexer
		indexer = 127;
	}
	// else if (light_sensor.get_value() < ball_brightness_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	// {
	// 	indexer = 0;
	// }
	else
	{
		indexer = 0;
	}
}
//sensors
void displayData()
{
	int line_number_to_set_text = 3;
	bool prev_B_button_value = false;
	while (true)
	{
		pros::lcd::set_text(0, "coordinates: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
		pros::lcd::set_text(1, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
		pros::lcd::set_text(2, "light sensor value: " + std::to_string(light_sensor.get_value()));
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && prev_B_button_value == false)
		{
			pros::lcd::set_text(line_number_to_set_text, "(" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")" + "; " + std::to_string(autonutils.get_alpha_in_degrees()));
			line_number_to_set_text++;
		}
		if (line_number_to_set_text > 5)
		{
			line_number_to_set_text = 3;
		}

		prev_B_button_value = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
		pros::Task::delay(20);
	}
	// // // pros::lcd::initialize();
	// // pros::lcd::set_text(1, "indexer motor temperature: " + std::to_string(indexer.get_temperature()));
	// // pros::lcd::set_text(7, "flywheel motor temperature: " + std::to_string(flywheel.get_temperature()));
	// // pros::lcd::set_text(4, "light sensor value: " + std::to_string(light_sensor.get_value()));
	// int prev_limit_value = 0;
	// int lower_balls_counted = 0;
	// int upper_balls_counted = 0;
	// int delta_light_sensor_value;
	// int prev_light_value = light_sensor.get_value();
	// int prev_time = pros::millis();
	// while (true)
	// {
	// 	if (limit_switch.get_value() == 1 && prev_limit_value == 0)
	// 	{
	// 		lower_balls_counted++;
	// 	}

	// 	// pros::lcd::set_text(6, "the limit switch value is: " + std::to_string(limit_value));
	// 	prev_limit_value = limit_switch.get_value();

	// 	int light_sensor_value = light_sensor.get_value();
	// 	delta_light_sensor_value = prev_light_value - light_sensor_value;
	// 	if (delta_light_sensor_value < -200 && pros::millis() - prev_time > 250)
	// 	{
	// 		upper_balls_counted++;
	// 		prev_time = pros::millis();
	// 	}
	// 	// pros::lcd::set_text(6, "the limit switch value is: " + std::to_string(limit_value));
	// 	prev_light_value = light_sensor_value;
}

void calibrateIMU()
{
	pros::lcd::set_text(5, "Calibrating IMU");
	IMU.reset();

	while (IMU.is_calibrating())
	{
		pros::delay(10);
	}
}

void color_sorting()
{
	pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
	pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);

	vision_sensor.set_signature(1, &BLUE_BALL_SIGNATURE);
	vision_sensor.set_signature(2, &RED_BALL_SIGNATURE);

	while (true)
	{
		pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
		// Gets the largest object of the EXAMPLE_SIG signature
		// pros::lcd::set_text(5, "signature: " + std::to_string(rtn.signature));
		// pros::lcd::set_text(3, "w: " + std::to_string(rtn.width));
		// pros::lcd::set_text(4, "h: " + std::to_string(rtn.height));
		pros::delay(10);
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// flywheel negative, manual dispensing
			flywheel = -127 / 2;
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			//flywheel positive, shooting
			flywheel = 127;
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			if (rtn.signature == 2) //red
			{
				flywheel = -127 / 2;
				indexer = 127;
				pros::delay(200);
			}
			else if (rtn.signature == 1) // blue
			{
				flywheel = 127;
			}
			else
			{
				flywheel = 127;
			}
		}
		else
		{
			flywheel = 0;
		}
		pros::delay(20);
	}
}

void opcontrol()
{
	pros::Task color_sorter(color_sorting);
	pros::Task balls_counted(displayData);
	while (true)
	{
		limit_switch_value();
		drive();
		run_macros();
		pros::delay(20);
	}
}
