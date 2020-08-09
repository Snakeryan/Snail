#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "globals.h"
#include "helper.h"
void calibrateIMU();

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	calibrateIMU();
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello World!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void competition_initialize() {}

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
	run_auton();
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
	if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		setIntake(127);
	}
	else 
	{
		setIntake(0);
	}
	if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		// indexer
		indexer = 127;
	} else 
	{
		indexer = 0;
	}
	
	
}
//sensors
void displayData()
{
	// pros::lcd::initialize();	
	pros::lcd::set_text(2, "Left: " + std::to_string(encoderL.get_value()));
	pros::lcd::set_text(3, "Right" + std::to_string(encoderR.get_value()));
	pros::lcd::set_text(4, "Middle" + std::to_string(encoderM.get_value()));
	pros::lcd::set_text(5, "IMU " + std::to_string(IMU.get_heading()));
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
		pros::lcd::set_text(2, "signature: " + std::to_string(rtn.signature));
		pros::lcd::set_text(3, "w: " + std::to_string(rtn.width));
		pros::lcd::set_text(4, "h: " + std::to_string(rtn.height));
		pros::delay(10);
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			// flywheel negative, manual dispensing
			flywheel = -127;
		} 
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			//flywheel positive, shooting
			flywheel = 127;
		}  
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			if(rtn.signature == 2) //red
			{
				flywheel = -127;
			}
			else if(rtn.signature == 1) // blue
			{
				flywheel = 127;
			} else 
			{
				flywheel = 0;
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
	while (true) 
	{
		drive();
		run_macros();
		pros::delay(20);
	}
}
