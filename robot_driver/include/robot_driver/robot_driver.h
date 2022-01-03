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

class RobotDriver
{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_scan_;

    tf::TransformListener listener_;

    float correction_threshold_ = 0.005;

    float max_speed_ = 1.0;
    float error_margin_ = 1.01359;

public:

    RobotDriver(ros::NodeHandle nh);

    void set_correction_threshold(float correction_threshold)
    {
        correction_threshold_ = correction_threshold;
    }

    float get_correction_threshold()
    {
        return correction_threshold_;
    }

    bool correct_angle();

    bool turn(bool clockwise, float radians);
    float drive(float distance);
};

void sayHello();
