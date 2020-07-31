#include "main.h"
#include "auton.h"
#include "auton_utils.h"


void run_auton() 
{ 
    pros::lcd::set_text(9, "autonomous started");
    point_turn_PID(160, 6.5, 0.03, -13);

}