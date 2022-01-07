/**
 * @file robot_driver.h
 * @author Louis Gasnault
 */

#pragma once

#define _USE_MATH_DEFINES

#include <utility>
#include <optional>
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "robot_driver/GetCorrection.h"

/**
 * @brief Class containing handy methods to control the robot movements.
 * 
 * @note Explain error_margin_ and other wizardery
 * 
 */

class RobotDriver
{
private:
    ros::NodeHandle nh_;                    /**< Used to check system state during loop and advertise publishers. */

    ros::Publisher pub_cmd_vel_;            /**< Used to publish geometry_msgs/Twist message to drive the robot. */

    tf::TransformListener listener_;        /**< Used to get current transform while turning. */

    float correction_threshold_ = 0.005;    /**< Below this, the correction is not applied (in radians). */

    float max_speed_ = 1.0;                 /**< Max linear speed of the ROSbot 2.0 Pro, see Husarion manual. */
    float error_margin_ = 1.01359;          /**< Factor to correct the error of RobotDriver::drive, see Note of RobotDriver class. */

public:

    /**
     * @brief Create a new RobotDriver.
     * 
     * @param nh Handle created in main function.
     */
    RobotDriver(ros::NodeHandle nh);

    /**
     * @param correction_threshold new correction threshold.
     */
    void set_correction_threshold(float correction_threshold)
    {
        correction_threshold_ = correction_threshold;
    }

    /**
     * @return Current correction threshold.
     */
    float get_correction_threshold()
    {
        return correction_threshold_;
    }

    /**
     * @brief Correct the orientation of the robot.
     * @details Makes requests to the get_correction server and calls the RobotDriver::turn method with the received angle.
     *  Continue until the server sends an angle bellow the correction threshold.
     *  
     * @return True if the correction was successful.
     */
    bool correct_angle();

    /**
     * @brief Rotates the robot with a given angle in radians.
     * @todo Explain the algorithm or link ressources
     * 
     * @param clockwise Indicates the rotation direction.
     * @param radians Angle to turn (in radians).
     * 
     * @return True if the rotation was successful.
     */
    bool turn(bool clockwise, float radians);

    /**
     * @brief Makes the robot move in a linear way.
     * @todo Explain the algorithm or link ressources
     * 
     * @param distance Distance to be traveled (in meter), can be negative.
     * @return 0 (real traveled distance not implemented yet)
     */
    float drive(float distance);
};

void sayHello();
