#pragma once

#include <iostream>
#include <thread>
#include <chrono>
#include <robot_main/MainConfig.h>
#include "robot_driver/robot_driver.h"

class Routine
{
private:
    RobotDriver &rd_;

    int steps_;
    float step_distance_;

public:
    Routine(RobotDriver &rd);
    Routine(RobotDriver &rd, int steps, float step_distance);

    void reconfigure(robot_main::GeneralConfig &config, uint32_t level);

    void set_steps(int steps)
    {
        steps_ = steps;
    }

    void set_step_distance(float step_distance)
    {
        step_distance_ = step_distance;
    }

    int get_steps()
    {
        return steps_;
    }

    int get_step_distance()
    {
        return step_distance_;
    }

    void start();
};