#include "robot_main/routine.h"

Routine::Routine(RobotDriver &rd)
    : Routine(rd, 0, 0)
{}

Routine::Routine(RobotDriver &rd, int steps, float step_distance)
    : rd_(rd)
{
    steps_ = steps;
    step_distance_ = step_distance;
}

void Routine::reconfigure(int steps, float step_distance)
{
    steps_ = steps;
    step_distance_ = step_distance;
}

void Routine::start()
{
    for (int current_step = 0; current_step < steps_; current_step++)
    {
        rd_.correct_angle();
        rd_.drive(step_distance_);

        std::cout << "Pause" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}