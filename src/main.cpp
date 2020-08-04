#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "globals.h"

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




void setIntake(int power)
{
    intakeleft = power;
    intakeright = power;

}
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
void runMacros()
{	
	if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		// launching
		indexer = 127;
		flywheel = 127;
		setIntake(127);
	}
	else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		// collecting
		indexer = 127;
		setIntake(127);
		// flywheel = 127 maybe do not want to include this and just run indexers at full speed
	}
	else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		// Dispensing
		indexer = 127;
		flywheel = -127;
		setIntake(127);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		//lower indexers
		indexer = -127/2; // may want to divide by 2 to make the balls go down slower
		flywheel = -127/2; //  may want to divide by 2 to make the balls go down slower
	}

	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
	{
		int deg = round(IMU.get_heading());
		point_turn_PID(deg, 7, 0, -10, false);
		pros::delay(20);
	}
	
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		if(IMU.get_heading() <= 45 || IMU.get_heading() > 315)
		{
			point_turn_PID(0, 7, 0, 0, false);
			drive();
			pros::delay(20);
		}
		else if(IMU.get_heading() > 45 && IMU.get_heading() <= 135)
		{
			point_turn_PID(90, 7, 0, 0, false);
			drive();
			pros::delay(20);
		}
		else if(IMU.get_heading() > 135 && IMU.get_heading() <= 225)
		{
			point_turn_PID(180, 7, 0, 0, false);
			drive();
			pros::delay(20);
		}
		else
		{
			point_turn_PID(270, 7, 0, 0, false);
			drive();
			pros::delay(20);
		}
	}
	
	else
	{

		int flywheelPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)));
		flywheel = flywheelPower;
		int indexerPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)));
		indexer = indexerPower;
		setIntake(0);
	}	
	// int intakePower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) - (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)));
    // setIntake(intakePower);
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


void opcontrol() 
{	
	while (true) 
	{
		drive();
		runMacros();
		displayData();
		pros::delay(20);
	}
}
