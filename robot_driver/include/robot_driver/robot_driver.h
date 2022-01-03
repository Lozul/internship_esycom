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

#define NO_SCAN_POINT 100
#define FIND_BORDERS_FAILED 101
#define NO_BORDERS 102

struct Point
{
    float range = 0;
    float angle = 0;
};

struct Target
{
    // Front points from scan where target should be
    std::vector<Point> points;

    // Indexes of target's edges
    int first_edge_index = 0;
    int second_edge_index = 0;

    // Misc data used to find target
    float range = 0;
    float theta_angle = 0;
};

struct CorrectionReport
{
    Target target;

    std::vector<float> polyfit;

    std::optional<sensor_msgs::LaserScanConstPtr> last_scan;

    bool success;
};

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

    CorrectionReport correct_angle();

    bool turn(bool clockwise, float radians);
    float drive(float distance);
};

void sayHello();

Target get_target(sensor_msgs::LaserScanConstPtr scan);
