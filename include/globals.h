#ifndef GLOBALS_H
#define GLOBALS_H
#include "Scorer.h"
#include "main.h"
#include "Drivetrain.h"

//namespace to allow for cycling through different autonmous modes
namespace auton_modes
{
    enum Auton_mode
    {
        home_row,
        skills,
    };
}

extern auton_modes::Auton_mode auton_mode;

//object of class autonUtils
extern DriveTrain drivetrain;

//object of class Scorer
extern Scorer scorer;

//motors:
extern pros::Motor intakeleft;
extern pros::Motor intakeright;
extern pros::Motor indexer;
extern pros::Motor flywheel;
extern pros::Motor FL;
extern pros::Motor FR;
extern pros::Motor BR;
extern pros::Motor BL;

//controller:

extern pros::Controller controller;

//encoders:

extern pros::ADIEncoder encoderL;
extern pros::ADIEncoder encoderM;
extern pros::ADIEncoder encoderR;

//sensors:
extern pros::Imu IMU;
extern pros::ADIDigitalIn lower_limit_switch;
extern pros::ADIAnalogIn light_sensor;

extern pros::Vision vision_sensor;
extern pros::vision_signature_s_t BLUE_BALL_SIGNATURE;
extern pros::vision_signature_s_t RED_BALL_SIGNATURE;

#endif