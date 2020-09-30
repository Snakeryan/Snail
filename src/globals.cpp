#include "main.h"
#include "Scorer.h"
#include "globals.h"



bool is_disabled;

//object of class DriveTrain:
DriveTrain drivetrain(1.375, 7.02739575, 7.02739575, 7.02739575, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM, &vision_sensor, &IMU, &left_pot, &right_pot);

//object of class Scorer:
Scorer scorer(&intakeleft, &intakeright, &indexer, &flywheel, &vision_sensor, &BLUE_BALL_SIGNATURE, &RED_BALL_SIGNATURE, &lower_limit_switch, &upper_counter_light_sensor, &dispense_counter_light_sensor);

//motors:
pros::Motor intakeleft(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeright(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor indexer(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor FL(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FR(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BR(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BL(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//controlller:

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::ADIEncoder encoderL('C', 'D', true);
pros::ADIEncoder encoderM('G', 'H', false);
pros::ADIEncoder encoderR('E', 'F', false);


pros::Imu IMU(10);

pros::Vision vision_sensor(6);
pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);
pros::ADIAnalogIn upper_counter_light_sensor({1, 'A'});

pros::ADIAnalogIn collision_light_sensor({1, 'H'});
pros::ADIAnalogIn dispense_counter_light_sensor({1, 'F'});

pros::ADIDigitalIn lower_limit_switch('B');

pros::ADIAnalogIn right_pot('A');
pros::ADIAnalogIn left_pot({1, 'B'});

//namespace to allow for cycling through different autonmous modes

auton_modes::Auton_mode auton_mode = auton_modes::skills;

//Miscellaneous:
/*
the odometry proofs are on page 40 of the vex notebook
*/