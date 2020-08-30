#include "Scorer.h"

Scorer::Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *light_sensor)
{
    this->intakeleft = intakeleft;
    this->intakeright = intakeright;
    this->indexer = indexer;
    this->flywheel = flywheel;
    this->vision_sensor = vision_sensor;
    this->BLUE_BALL_SIGNATURE = BLUE_BALL_SIGNATURE;
    this->RED_BALL_SIGNATURE = RED_BALL_SIGNATURE;
    this->lower_limit_switch = lower_limit_switch;
    this->light_sensor = light_sensor;
}

void Scorer::run_lower_limit_switch()
{
    // makes the upper_balls_counted only go up when it is greater than 100 milliseconds:
    double delay_time = 100;

    // making a variable that stores the previous value of the limit switch:

    if ((lower_limit_switch->get_value() == 1 && prev_lower_limit_value == 0) && abs(lower_prev_time - pros::millis()) > delay_time)
    {
        lower_balls_counted++;
        lower_prev_time = pros::millis();
    }

    prev_lower_limit_value = lower_limit_switch->get_value();
}

void Scorer::run_upper_light_sensor()
{
    // makes the upper_balls_counted only go up when it is greater than 100 milliseconds:
    double delay_time = 100;

    // value that the light sensor crosses to know if a ball has exited its field of detection:
    double light_sensor_threshold = -500;

    // logic to make an upper counting system for balls that have exited our loop:
    if ((get_light_calibrated_value() > light_sensor_threshold && prev_upper_light_value < light_sensor_threshold) && abs(upper_prev_time - pros::millis()) > delay_time)
    {
        upper_balls_counted++;
        upper_prev_time = pros::millis();
    }

    // setting the previous light value:
    prev_upper_light_value = get_light_calibrated_value();
}

void Scorer::manage_dispensing()
{
    if (dispense_triggered)
    {
        set_indexers(-127);
        wait_until_number_of_lower_balls_counted(lower_balls_counted + 1);
        set_indexers(127);
        set_flywheel(-127);
        dispense_triggered = false;
    }
}

void Scorer::start_dispense_management_thread()
{
    while (true)
    {
        manage_dispensing();
        pros::Task::delay(20);
    }
}

void Scorer::start_auton_sensors_update_thread()
{
    while (true)
    {
        run_upper_light_sensor();
        run_lower_limit_switch();
        pros::Task::delay(20);
    }
}
// void Scorer::start_intake_manager_thread()
// {
//     while (true)
//     {
//         //TODO Later
//         pros::Task::delay(50);
//     }
// }

void Scorer::make_scorer_threads()
{
    flywheel_and_indexer_manager_task = std::make_shared<pros::Task>([this] { start_dispense_management_thread(); });
    auton_sensors_task = std::make_shared<pros::Task>([this] { start_auton_sensors_update_thread(); });
    // intake_manager_task = std::make_shared<pros::Task>([this] { start_intake_manager_thread(); });
}

void Scorer::set_intakes(int power)
{
    intakeleft->move_voltage(power * 1000);
    intakeright->move_voltage(power * 1000);
}

void Scorer::set_flywheel(int flywheel_power)
{
    flywheel->move_voltage(flywheel_power * 1000);
}

void Scorer::set_indexers(int indexer_power)
{
    indexer->move_voltage(indexer_power * 1000);
}

double Scorer::get_light_calibrated_value()
{
    return light_sensor->get_value() - light_sensor_calibrated_value;
}

void Scorer::wait_until_number_of_uppper_balls_counted(int number_of_balls_passed)
{
    while (upper_balls_counted < number_of_balls_passed)
    {
        pros::delay(20);
    }
}

void Scorer::wait_until_number_of_lower_balls_counted(int number_of_balls_passed)
{
    while (lower_balls_counted < number_of_balls_passed)
    {
        pros::delay(20);
    }
}

void Scorer::score_in_goal(int num_balls)
{
    set_flywheel(127);
    pros::delay(100);
    set_indexers(127);
    while (upper_balls_counted < num_balls)
    {
        pros::delay(10);
    }
    pros::delay(250);
    set_flywheel(127);
    set_indexers(127);
}

void Scorer::score_in_goal_with_light(int num_balls)
{
    set_flywheel(127);
    set_indexers(127);

    while (upper_balls_counted < num_balls)
    {
        if (get_light_calibrated_value() < -500)
        {
            set_indexers(-100);
            pros::delay(50);
        }
        else
        {
            set_indexers(127);
        }
        pros::delay(10);
    }
    pros::delay(100);
    set_flywheel(0);
    set_indexers(0);
}

void Scorer::dispense()
{
    dispense_triggered = true;
}

void Scorer::reset_balls_counted()
{
    upper_balls_counted = 0;
    lower_balls_counted = 0;
}

void Scorer::deploy_intakes()
{
    set_intakes(127);
    pros::delay(1000);
    set_intakes(0);
    pros::delay(1000);
    set_intakes(127);
}

void Scorer::stop_motors()
{
    set_flywheel(0);
    set_indexers(0);
    set_intakes(0);
}

void Scorer::setup()
{
    light_sensor_calibrated_value = light_sensor->calibrate();
    make_scorer_threads();
}

Scorer::~Scorer()
{
    if (flywheel_and_indexer_manager_task.get() != nullptr)
    {
        flywheel_and_indexer_manager_task->remove();
    }
    if (auton_sensors_task.get() != nullptr)
    {
        auton_sensors_task->remove();
    }
    // if (intake_manager_task.get() != nullptr)
    // {
    //     intake_manager_task->remove();
    // }
}
