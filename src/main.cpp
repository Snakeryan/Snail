#include "main.h"
#include "Scorer.h"
#include "auton.h"
#include "globals.h"
#include "Drivetrain.h"
#include "globals.h"
#include "autoSelect/selection.h"
#include "pros/rtos.hpp"

void display_data()
{
	pros::vision_object_s_t backboard = vision_sensor.get_by_size(0);
	while (true)
	{
		backboard = vision_sensor.get_by_size(0);

		// odometry coordinates and the robot's heading:
		pros::lcd::set_text(1, "(X, Y): (" + std::to_string(drivetrain.get_globalX()) + ", " + std::to_string(drivetrain.get_globalY()) + ")");
		// pros::lcd::set_text(2, "alpha: " + std::to_string(drivetrain.get_alpha_in_degrees()));
		// pros::lcd::set_text(4, "i: " + std::to_string(IMU.get_heading()) + "a: " + std::to_string(drivetrain.get_alpha_in_degrees()));
		// pros::lcd::set_text(4, "i norm: " + std::to_string(IMU.get_heading()) + "a: " + std::to_string(drivetrain.get_alpha_in_degrees()));

		// light sensor values:
		pros::lcd::set_text(2, "lower light counter: " + std::to_string(scorer.get_lower_balls_counted()));
		pros::lcd::set_text(3, "lower light: " + std::to_string(scorer.get_lower_light_calibrated_value()));
		// pros::lcd::set_text(5, "upper light: " + std::to_string(scorer.get_upper_light_calibrated_value()));
		// pros::lcd::set_text(2, "collision light: " + std::to_string(collision_light_sensor.get_value_calibrated()));

		// motor temperatures:
		pros::lcd::set_text(7, std::to_string((int)FL.get_temperature()) + "; " + std::to_string((int)FR.get_temperature()) + "; " + std::to_string((int)BL.get_temperature()) + "; " + std::to_string((int)BR.get_temperature()));
		pros::lcd::set_text(0, "indexer:" + std::to_string((int)indexer.get_temperature()) + "; flywheel:" + std::to_string((int)flywheel.get_temperature()));

		//ball counters:
		// pros::lcd::set_text(6, "upper_balls: " + std::to_string(scorer.get_upper_balls_counted()));
		pros::lcd::set_text(1, "balls_to_score: " + std::to_string(scorer.get_balls_to_score()));

		// encoders:
		// double calculation = (drivetrain.get_left_encoder_distance() - drivetrain.get_right_encoder_distance()) / (20 * pi);
		pros::lcd::set_text(5, "M_encoder: " + std::to_string(drivetrain.get_middle_encoder_distance()));
		// pros::lcd::set_text(6, "L_encoder: " + std::to_string(drivetrain.get_left_encoder_distance()));
		pros::lcd::set_text(7, "R_encoder: " + std::to_string(drivetrain.get_right_encoder_distance()));
		// pros::lcd::set_text(2, "calculation: " + std::to_string(calculation));

		// potentiometer values:
		// pros::lcd::set_text(6, "pot_L: " + std::to_string(left_pot.get_value()));
		// pros::lcd::set_text(5, "pot_R :" + std::to_string(right_pot.get_value()));

		// vision object x position
		// pros::lcd::set_text(1, "vision X coordinate:" + std::to_string(backboard.x_middle_coord));
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

void calibrate_sensors()
{
	upper_counter_light_sensor.calibrate();
	dispense_counter_light_sensor.calibrate();
	middle_light_sensor.calibrate();
	drivetrain.calibrate_IMU();
}

void initialize()
{
	pros::lcd::initialize();
	scorer.setup();
	drivetrain.setup();
	pros::Task display_data_task(display_data);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task stop_watch(start_stop_watch);
	display_auton_mode();
	calibrate_sensors();
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
	const int upper_light_sensor_threshold = 1200, middle_light_sensor_threshold = 2200;
	int prev_y = 0;
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		scorer.set_intakes(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		scorer.set_intakes(-63);
	}
	else
	{
		scorer.set_intakes(0);
	}


    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == 1 && prev_y == 0)
    {
		run_skills();
    }

	// If both buttons are pressed and limit switch is detected: stop indexer
	// If only B is pressed: reverse indexer
	// If only L2 is pressed: indexer
	// Else: stop indexer

	if (((scorer.get_upper_light_calibrated_value() < upper_light_sensor_threshold || scorer.get_middle_light_calibratred_value() < middle_light_sensor_threshold) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		// scorer.set_indexers(0);
		double delay_time = pros::millis() + 150;
    	while (pros::millis() < delay_time)
    	{
        	if (scorer.get_upper_light_calibrated_value() < 1200)
        	{
         		break;
        	}
		}
		// scorer.set_flywheel(-30);
		pros::delay(50);
	}
	// else if((scorer.get_upper_light_calibrated_value() < upper_light_sensor_threshold && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))
	// {
	// 	double delay_time = pros::millis() + 150;
    // 	while (pros::millis() < delay_time)
    // 	{
    //     	if (scorer.get_upper_light_calibrated_value() < 1200)
    //     	{
    //      		break;
    //     	}
    // 	}
	// 	scorer.set_flywheel(-30)
	// 	pros::delay(50);
	// }
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		scorer.set_indexers(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
	{
		scorer.set_indexers(-127);
	}

	else
	{
		scorer.set_indexers(0);
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
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
	{
		printf("drivetrain.drive_to_point(%.2f, %.2f, %.2f, false, true);\n", drivetrain.get_globalX(), drivetrain.get_globalY(), drivetrain.get_alpha_in_degrees());
		// (4, "drivetrain.drive_to_point( " + std::to_string(drivetrain.get_globalX()) + ", " + std::to_string(drivetrain.get_globalY()) + ")" + std::to_string(drivetrain.get_alpha_in_degrees()));
		pros::delay(200);
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
	{
		printf("drivetrain.drive_to_point(%.2f, %.2f, %.2f, false, false);\n", drivetrain.get_globalX(), drivetrain.get_globalY(), drivetrain.get_alpha_in_degrees());
		pros::delay(200);
	}
	if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		controller.print(0, 0, "(%.2f, %.2f)", drivetrain.get_globalX(), drivetrain.get_globalY());
		pros::delay(50);
		controller.print(1, 0, "A: %.2f I: %.2f", drivetrain.get_alpha_in_degrees(), drivetrain.get_IMU_heading());
		pros::delay(50);
		controller.print(2, 0, "Battery: %.2f", controller.get_battery_level());
		pros::delay(50);
	}
	else
	{
		controller.clear();
	}
}

void opcontrol()
{
	// pros::lcd::initialize();
	// drivetrain.set_current_global_position(16.077751092179696, 63.107507307056174, 90);
	while (true)
	{
		drivetrain.driver_control(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
								  controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
								  controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		run_macros();
		// pros::lcd::set_text(5, std::to_string(pros::millis()));
		pros::delay(20);
	}
}