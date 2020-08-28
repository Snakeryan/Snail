#ifndef SCORER_H
#define SCORER_H

#include "main.h"

class Scorer
{

    // counters for the balls entering and exiting our loop and upper balls:
    int lower_balls_counted, upper_balls_counted = 0;

    //calibrated value of the light sensor:
    double light_sensor_calibrated_value = 0;

    //pointer objects of all of the motors that are not part of the drivetrain:
    pros::Motor *intakeleft;
    pros::Motor *intakeright;
    pros::Motor *indexer;
    pros::Motor *flywheel;

    //pointer object of the vision sensor:
    pros::Vision *vision_sensor;

    //pointer objects of all of our vision signatures:
    pros::vision_signature_s_t *BLUE_BALL_SIGNATURE;
    pros::vision_signature_s_t *RED_BALL_SIGNATURE;
    pros::vision_signature_s_t *tower_backboard_signature;

    //pointers to the three-wire sensors:
    pros::ADIDigitalIn *lower_limit_switch;
    pros::ADIAnalogIn *light_sensor;

public:
    Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::vision_signature_s_t *tower_backboard_signature, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *light_sensor);

    void set_intake(int power);

    double get_light_calibrated_value();

    ~Scorer();
};

#endif