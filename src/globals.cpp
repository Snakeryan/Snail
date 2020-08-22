#include "main.h"

//variable to know if you want to auto sort balls
bool auto_sort_balls = false;

//object of class autonUtils
AutonUtils autonutils(1.375, 6.9922569449, 6.9922569449, 6.9922569449, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM);

//motors:
pros::Motor intakeleft(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeright(20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor indexer(3, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(12, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor FL(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FR(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BR(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BL(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//controlller:

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::ADIEncoder encoderL('A', 'B', false);
pros::ADIEncoder encoderM('H', 'G', false);
pros::ADIEncoder encoderR('E', 'F', true);

pros::Imu IMU(2);

pros::Vision vision_sensor(9);
pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);

pros::ADIDigitalIn lower_limit_switch('C');
pros::ADIDigitalIn upper_limit_switch('D');

//namespace to allow for cycling through different autonmous modes

auton_modes::Auton_mode auton_mode = auton_modes::skills;

//Miscellaneous:
/*
the odometry proofs are on page 40 of the vex notebook
*/