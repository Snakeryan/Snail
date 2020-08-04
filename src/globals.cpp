

#include "main.h"

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
pros::ADIEncoder encoderL ('A', 'B', false);
pros::ADIEncoder encoderM ('H', 'G', false);
pros::ADIEncoder encoderR ('F', 'E', true);
pros::Imu IMU(2);

//Miscellaneous:
/*
the odometry proofs are on page 40 of the vex notebook
*/