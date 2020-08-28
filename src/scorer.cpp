#include "scorer.h"

Scorer::Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::vision_signature_s_t *tower_backboard_signature, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *light_sensor)
{
    this->intakeleft = intakeleft;
    this->intakeright = intakeright;
    this->indexer = indexer;
    this->flywheel = flywheel;
    this->vision_sensor = vision_sensor;
    this->BLUE_BALL_SIGNATURE = BLUE_BALL_SIGNATURE;
    this->RED_BALL_SIGNATURE = RED_BALL_SIGNATURE;
    this->tower_backboard_signature = tower_backboard_signature;
    this->lower_limit_switch = lower_limit_switch;
    this->light_sensor = light_sensor;
}

void Scorer::set_intake(int power)
{
    intakeleft->move_voltage(power * 1000);
    intakeright->move_voltage(power * 1000);
}

double Scorer::get_light_calibrated_value()
{
    return light_sensor->get_value() - light_sensor_calibrated_value;
}

