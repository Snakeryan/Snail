#include "main.h"
#include "helper.h"

void setIntake(int power)
{
    intakeleft = power;
    intakeright = power;
}

void limit_switch_value()
{
    static int prev_limit_value = 0;
    static int balls_counted = 0;
    if (limit_switch.get_value() == 1 && prev_limit_value == 0)
    {
        balls_counted++;
    }
    pros::lcd::set_text(6, "the limit switch value is: " + std::to_string(balls_counted));
    prev_limit_value = limit_switch.get_value();
}
