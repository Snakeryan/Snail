#include "Scorer.h"

Scorer::Scorer(pros::Motor *intakeleft, pros::Motor *intakeright, pros::Motor *indexer, pros::Motor *flywheel, pros::Vision *vision_sensor, pros::vision_signature_s_t *BLUE_BALL_SIGNATURE, pros::vision_signature_s_t *RED_BALL_SIGNATURE, pros::ADIDigitalIn *lower_limit_switch, pros::ADIAnalogIn *upper_counter_light_sensor, pros::ADIAnalogIn *dispense_counter_light_sensor)
{
    this->intakeleft = intakeleft;
    this->intakeright = intakeright;
    this->indexer = indexer;
    this->flywheel = flywheel;
    this->vision_sensor = vision_sensor;
    this->lower_limit_switch = lower_limit_switch;
    this->upper_counter_light_sensor = upper_counter_light_sensor;
    this->dispense_counter_light_sensor = dispense_counter_light_sensor;
}

//will eventually want to move this to its own class
void Scorer::setup_light_counter(double light_sensor_threshold, double light_calibrated_value, double counter)
{
    // makes the upper_balls_counted only go up when it is greater than 100 milliseconds:
    double delay_time = 100;

    // logic to make an upper counting system for balls that have exited our loop:
    if ((light_calibrated_value > light_sensor_threshold && prev_light_value < light_sensor_threshold) && std::abs(prev_time - pros::millis()) > delay_time)
    {
        counter++;
        prev_time = pros::millis();
    }

    // setting the previous light value:
    prev_light_value = light_calibrated_value;
}

void Scorer::run_lower_limit_switch()
{
    // makes the upper_balls_counted only go up when it is greater than 100 milliseconds:
    double delay_time = 100;

    // making a variable that stores the previous value of the limit switch:

    if ((lower_limit_switch->get_value() == 1 && prev_lower_limit_value == 0) && std::abs(lower_prev_time - pros::millis()) > delay_time)
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
    double light_sensor_threshold = 1200;

    // logic to make an upper counting system for balls that have exited our loop:
    if ((get_upper_light_calibrated_value() > light_sensor_threshold && prev_upper_light_value < light_sensor_threshold) && std::abs(upper_prev_time - pros::millis()) > delay_time)
    {
        upper_balls_counted++;
        upper_prev_time = pros::millis();
    }

    // setting the previous light value:
    prev_upper_light_value = get_upper_light_calibrated_value();
}

void Scorer::run_dispense_light_sensor()
{
    // makes the upper_balls_counted only go up when it is greater than 100 milliseconds:
    double delay_time = 100;

    // value that the light sensor crosses to know if a ball has exited its field of detection:
    double light_sensor_threshold = 1600;

    // logic to make an upper counting system for balls that have exited our loop:
    if ((get_dispense_light_calibrated_value() > light_sensor_threshold && prev_dispense_light_value < light_sensor_threshold) && std::abs(dispense_prev_time - pros::millis()) > delay_time)
    {
        dispense_balls_counted++;
        dispense_prev_time = pros::millis();
    }

    // setting the previous light value:
    prev_dispense_light_value = get_dispense_light_calibrated_value();
}

void Scorer::manage_indexer_and_flywheel()
{
    if (num_balls_to_dispense != 0)
    {
        // set_indexers(-127);
        // wait_until_number_of_lower_balls_counted(lower_balls_counted + 1);
        set_indexers(127);
        set_flywheel(-127);
        wait_until_number_of_balls_dispensed(num_balls_to_dispense);
        num_balls_to_dispense = 0;
    }

    if (num_balls_to_score != 0)
    {
        score_in_goal_with_light(num_balls_to_score);
        num_balls_to_score = 0;
    }

    if (num_balls_to_collect != 0 && num_balls_to_dispense == 0)
    {
        set_indexers(127);
    }
}

void Scorer::start_indexer_and_flywheel_management_thread()
{
    while (true)
    {
        manage_indexer_and_flywheel();
        pros::Task::delay(20);
    }
}

void Scorer::start_auton_sensors_update_thread()
{
    while (true)
    {
        run_upper_light_sensor();
        run_lower_limit_switch();
        run_dispense_light_sensor();
        pros::Task::delay(20);
    }
}

void Scorer::manage_intakes()
{
    if (num_balls_to_collect != 0 && num_balls_to_dispense_through_intakes == 0)
    {
        set_intakes(127);
        wait_until_number_of_lower_balls_counted(lower_balls_counted + num_balls_to_collect);
        pros::delay(50);
        set_intakes(-63);
        pros::delay(100);
        set_intakes(0);
        num_balls_to_collect = 0;
    }

    if (num_balls_to_dispense_through_intakes != 0)
    {
        set_flywheel(-127);
        set_indexers(-127);
        pros::delay(100);
        set_intakes(-127 / 5);
        wait_until_number_of_balls_dispensed(num_balls_to_dispense_through_intakes, true);
        num_balls_to_dispense_through_intakes = 0;
    }
}

void Scorer::start_intake_manager_thread()
{
    while (true)
    {
        manage_intakes();
        pros::Task::delay(40);
    }
}

void Scorer::make_scorer_threads()
{
    flywheel_and_indexer_manager_task = std::make_shared<pros::Task>([this] { start_indexer_and_flywheel_management_thread(); });
    auton_sensors_task = std::make_shared<pros::Task>([this] { start_auton_sensors_update_thread(); });
    intake_manager_task = std::make_shared<pros::Task>([this] { start_intake_manager_thread(); });
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

void Scorer::score_in_goal_with_light(int num_balls)
{
    set_flywheel(127);
    set_indexers(127);

    while (upper_balls_counted < num_balls)
    {
        if (get_upper_light_calibrated_value() < 1200)
        {
            set_indexers(0);
            pros::delay(50);
        }
        else
        {
            set_indexers(127);
        }
        pros::delay(10);
    }
    pros::delay(250);
    set_flywheel(0);
    set_indexers(0);
}

void Scorer::dispense_n_balls(double num_balls_to_dispense, bool is_intake_dispense)
{
    if (is_intake_dispense)
    {
        num_balls_to_dispense_through_intakes = num_balls_to_dispense;
    }
    else
    {
        this->num_balls_to_dispense = num_balls_to_dispense;
    }
}

void Scorer::wait_until_number_of_balls_dispensed(int num_balls_to_dispense, bool is_intake_dispense)
{
    if (is_intake_dispense)
    {
        wait_until_number_of_lower_balls_counted(num_balls_to_dispense);
    }
    else
    {
        while (dispense_balls_counted < num_balls_to_dispense)
        {
            pros::delay(20);
        }
    }
}

void Scorer::reset_balls_counted()
{
    upper_balls_counted = 0;
    lower_balls_counted = 0;
    dispense_balls_counted = 0;
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
    make_scorer_threads();
}

void Scorer::score_n_balls(double num_balls_to_score)
{
    this->num_balls_to_score = num_balls_to_score;
}

void Scorer::collect_n_balls(double num_balls_to_collect)
{
    this->num_balls_to_collect = num_balls_to_collect;
}

double Scorer::get_lower_balls_counted()
{
    return lower_balls_counted;
}

double Scorer::get_upper_balls_counted()
{
    return upper_balls_counted;
}

double Scorer::get_dispense_balls_counted()
{
    return dispense_balls_counted;
}

double Scorer::get_dispense_light_calibrated_value()
{
    return dispense_counter_light_sensor->get_value_calibrated();
}

double Scorer::get_upper_light_calibrated_value()
{
    return upper_counter_light_sensor->get_value_calibrated();
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
    if (intake_manager_task.get() != nullptr)
    {
        intake_manager_task->remove();
    }
}
