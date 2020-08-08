#include "main.h"
#include "auton.h"
#include "auton_utils.h"


AutonUtils autonutils(1.375, 6.86024, 6.86024, 6.86024, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM);

void run_auton() 
{ 
    autonutils.make_update_thread();
    autonutils.set_current_global_position(10, 10, 0);
    autonutils.drive_to_point(-10, -10, 180, true);
    FL = 0;
    FR = 0;
    BL = 0;
    BR = 0;
    // autonutils.turn_to_point(10, 0);
    while(true)
    {
        pros::lcd::set_text(1, "alpha: " + std::to_string(autonutils.get_alpha_in_degrees()));
        // pros::lcd::set_text(2, "IMU:" + std::to_string(IMU.get_heading()));
        // pros::lcd::set_text(3, "Right tracking wheel: " + std::to_string(autonutils.get_right_encoder_distance()));
        // pros::lcd::set_text(4, "Left tracking wheel: " + std::to_string(autonutils.get_left_encoder_distance()));
        // pros::lcd::set_text(5, "middle tracking wheel: " + std::to_string(autonutils.get_middle_encoder_distance()));
        pros::lcd::set_text(2, "globalX: " + std::to_string(autonutils.get_globalX()));
        pros::lcd::set_text(3, "globalY: " + std::to_string(autonutils.get_globalY()));
    //     // pros::lcd::set_text(6, "dlX: " + std::to_string(autonutils.get_dlX()));
    //     // pros::lcd::set_text(7, "dlY: " + std::to_string(autonutils.get_dlY()));
    //     // pros::lcd::set_text(6, "delta globalX: " + std::to_string(autonutils.get_delta_globalX()));
    //     // pros::lcd::set_text(7, "delta globalY: " + std::to_string(autonutils.get_delta_globalY()));

        pros::delay(20);
    }


    // point_turn_PID(160, 7, 0.01, -17); // 0.01 precision
    // point_turn_PID(90, 9, 0, 0);
}