/**
 * @file routine.h
 * @author Louis Gasnault
 */

#pragma once

/**
 * @brief Describe a routine to be executed by the robot
 */

struct Routine
{
    int steps;              /**< Number of steps */
    float step_distance;    /**< Distance between each step */
    float frequency;        /**< Frequency of the generator */
    float power_level;      /**< Power level of the generator */
};
