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

// class Routine
// {
// private:
//     RobotDriver &rd_;

//     int steps_;
//     float step_distance_;

// public:
//     Routine(RobotDriver &rd);
//     Routine(RobotDriver &rd, int steps, float step_distance);

//     void reconfigure(int steps, float step_distance);

//     void set_steps(int steps)
//     {
//         steps_ = steps;
//     }

//     void set_step_distance(float step_distance)
//     {
//         step_distance_ = step_distance;
//     }

//     int get_steps()
//     {
//         return steps_;
//     }

//     int get_step_distance()
//     {
//         return step_distance_;
//     }

//     void start();
// };