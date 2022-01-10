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
 */

class RobotDriver
{
private:
    ros::NodeHandle nh_;                    /**< Used to check system state during loop and advertise publishers. */

    ros::Publisher pub_cmd_vel_;            /**< Used to publish geometry_msgs/Twist message to drive the robot. */

    tf::TransformListener listener_;        /**< Used to get current transform while turning. */

    float correction_threshold_ = 0.005;    /**< Below this, the correction is not applied (in radians). */

    float max_speed_ = 1.0;                 /**< Max linear speed of the ROSbot 2.0 Pro, see Husarion manual. */
    float error_margin_ = 1.01359;          /**< Factor to correct the error of RobotDriver::drive. */

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
     * @note See \ref GetCorrectionExplained for details.
     *  
     * @return True if the correction was successful.
     */
    bool correct_angle();

    /**
     * @brief Rotates the robot with a given angle in radians.
     * @details Based on the tutorial for the ROS package `pr2_controllers`.
     *  Records the transform at the start, then sends Twist messages with
     *  an angular speed following a v-shaped function to ensure smooth movement.
     *  
     * @note For more details, see inline comments in `robot_driver.cpp`.
     * 
     * @param clockwise Indicates the rotation direction.
     * @param radians Angle to turn (in radians).
     * 
     * @return True if the rotation was successful.
     */
    bool turn(bool clockwise, float radians);

    /**
     * @brief Makes the robot move in a linear way.
     * @details Sets a maximum speed and determine total time to travel the given distance.
     *  Sends Twist messages with a linear speed following a trapezoidal function to ensure smooth movement.
     *  
     * @note For more details, see inline comments in `robot_driver.cpp`.
     * 
     * @param distance Distance to be traveled (in meter), can be negative.
     * @return 0 (real traveled distance not implemented yet)
     */
    float drive(float distance);
};

void sayHello();
