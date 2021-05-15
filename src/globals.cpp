#include "main.h"
#include "Scorer.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/motors.h"

bool is_disabled;

//object of class DriveTrain:
Drivetrain drivetrain(1.375, 7.264035, 7.264035, 0.625, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM, &vision_sensor, &IMU, &left_pot, &right_pot, &scorer);

//object of class Scorer:
Scorer scorer(&intakeleft, &intakeright, &indexer, &flywheel, &vision_sensor, &BLUE_BALL_SIGNATURE, &RED_BALL_SIGNATURE, &lower_counter_light_sensor, &upper_counter_light_sensor, &dispense_counter_light_sensor, &middle_light_sensor);



// motors:
pros::Motor intakeleft(11, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeright(18, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor indexer(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor FL(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FR(9, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BR(10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BL(2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//controller:

pros::Controller controller1(pros::E_CONTROLLER_MASTER);
pros::Controller controller2(pros::E_CONTROLLER_PARTNER);

//Sensors
pros::Rotation encoderL(12);
pros::Rotation encoderM(8);
pros::Rotation encoderR(19);

pros::Imu IMU(13);


pros::ADIAnalogIn upper_counter_light_sensor('A');
pros::ADIAnalogIn lower_counter_light_sensor('B');
pros::ADIAnalogIn dispense_counter_light_sensor('C');


pros::ADIAnalogIn right_pot('D');
pros::ADIAnalogIn left_pot('G');

pros::Vision vision_sensor(6);
pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);
pros::ADIAnalogIn middle_light_sensor('E');
pros::ADIAnalogIn collision_light_sensor('H');
/*
// Previous robot globals:

//object of class DriveTrain:
Drivetrain drivetrain(1.375, 7.02739575, 7.02739575, 7.02739575, &FL, &FR, &BL, &BR, &encoderL, &encoderR, &encoderM, &vision_sensor, &IMU, &left_pot, &right_pot, &scorer);

//object of class Scorer:
Scorer scorer(&intakeleft, &intakeright, &indexer, &flywheel, &vision_sensor, &BLUE_BALL_SIGNATURE, &RED_BALL_SIGNATURE, &lower_counter_light_sensor, &upper_counter_light_sensor, &dispense_counter_light_sensor, &middle_light_sensor);

// motors:
pros::Motor intakeleft(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeright(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor indexer(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor flywheel(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

pros::Motor FL(4, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor FR(7, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BR(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor BL(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//controller:

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::ADIEncoder encoderL('C', 'D', false);
pros::ADIEncoder encoderM('G', 'H', false);
pros::ADIEncoder encoderR('E', 'F', false);

pros::Imu IMU(10);

pros::Vision vision_sensor(6);
pros::vision_signature_s_t BLUE_BALL_SIGNATURE = pros::Vision::signature_from_utility(1, -2527, -1505, -2016, 6743, 11025, 8884, 1.500, 0);
pros::vision_signature_s_t RED_BALL_SIGNATURE = pros::Vision::signature_from_utility(2, 3571, 7377, 5474, -1, 541, 270, 1.000, 0);

pros::ADIAnalogIn upper_counter_light_sensor({1, 'A'});
pros::ADIAnalogIn middle_light_sensor({1, 'D'});

pros::ADIAnalogIn collision_light_sensor({1, 'H'});
pros::ADIAnalogIn dispense_counter_light_sensor({1, 'F'});

pros::ADIAnalogIn lower_counter_light_sensor('B');

pros::ADIAnalogIn right_pot('A');
pros::ADIAnalogIn left_pot({1, 'B'});
*/
//namespace to allow for cycling through different autonmous modes

auton_modes::Auton_mode auton_mode = auton_modes::skills;

//Miscellaneous:
/*
the odometry proofs are on page 40 of the vex notebook
*/