#pragma once

struct Routine
{
    int steps;
    float step_distance;

    void reconfigure(int steps_, float step_distance_)
    {
        steps = steps_;
        step_distance = step_distance_;
    }
};
