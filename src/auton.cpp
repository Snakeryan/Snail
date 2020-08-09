#include "main.h"
#include "auton.h"
#include "auton_utils.h"
#include "helper.h"

bool sort_balls = false;

AutonUtils autonutils(1.375, 6.86024, 6.86024, 6.86024, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM);

void score(int flywheel_power, int indexer_power)
{
    flywheel = flywheel_power;
    indexer = indexer_power;
}

void auton_color_sorting()
{
  	pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
  	pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);

  	vision_sensor.set_signature(1, &BLUE_BALL_SIGNATURE);
	vision_sensor.set_signature(2, &RED_BALL_SIGNATURE);
	
	while (true) 
	{
        if (sort_balls) 
        {
            pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
            // Gets the largest object of the EXAMPLE_SIG signature
            pros::lcd::set_text(2, "signature: " + std::to_string(rtn.signature));
            pros::lcd::set_text(3, "w: " + std::to_string(rtn.width));
            pros::lcd::set_text(4, "h: " + std::to_string(rtn.height));
            pros::delay(10);
            
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
        pros::Task::delay(10);
		
	}
}

void debug_autonomous()
{
    while(true)
    {
        pros::lcd::set_text(1, "coordinates: (" + std::to_string(autonutils.get_globalX()) + ", " + std::to_string(autonutils.get_globalY()) + ")");
        pros::lcd::set_text(2, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
        pros::Task::delay(10);
    
    }

}

void stop()
{
    FL = 0;
    FR = 0;
    BL = 0;
    BR = 0;
}

void run_auton() 
{ 
    autonutils.make_update_thread();
    pros::Task color_sorter(auton_color_sorting);
    pros::Task autonomous_debugger(debug_autonomous);
    
    pros::lcd::set_text(0, "Autonomous Mode ");
    autonutils.set_current_global_position(0, 0, 0);

    setIntake(127);
    autonutils.drive_to_point(2, -17, 45, false, true);

    // First Goal
    autonutils.drive_to_point(16.2, -5.7, 45, false, true);
    stop();
    sort_balls = true;
    score(127, 127);
    pros::delay(2000);

    // Waypoint to second goal
    sort_balls = false;
    score(0, 0);
    setIntake(0);
    autonutils.drive_to_point(-37.1, -23.7, 0, false, true);
    autonutils.drive_to_point(-37.5, -9.6, 0, false, false);
    stop();

}
