#ifndef GLOBALS_H
#define GLOBALS_H
#include "main.h"

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
extern pros::ADIDigitalIn limit_switch;

extern pros::Vision vision_sensor;

// extern bool is_autonomous;
#endif 