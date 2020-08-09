

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

// pros::vision_signature BLUE_BALL_SIGNATURE (1, -2821, -619, -1720, 1983, 6425, 4204, 1.200, 0); //BLUE
// pros::vision_signature RED_BALL_SIGNATURE (1, 7209, 10415, 8812, -2529, -2115, -2322, 1.700, 0);
pros::Vision vision_sensor(4);


//Miscellaneous:
/*
the odometry proofs are on page 40 of the vex notebook
*/