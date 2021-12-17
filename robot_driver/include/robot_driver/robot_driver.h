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
#include "polyfit/PolyfitBoost.hpp"
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

    float scan_range_;
    float correction_threshold_;
    float turn_speed_;
    float drive_speed_;
    float target_distance_;

    float max_drive_speed_ = 1.0;
    float max_turn_speed_ = 7.33;
    float error_margin_ = 1.01359;

public:
    RobotDriver(ros::NodeHandle nh);
    RobotDriver(ros::NodeHandle nh, float scan_range, float correction_threshold, float turn_speed, float drive_speed, float target_distance);

    void set_scan_range(float scan_range)
    {
        scan_range_ = scan_range;
    }

    void set_correction_threshold(float correction_threshold)
    {
        correction_threshold_ = correction_threshold;
    }

    void set_turn_speed(float turn_speed)
    {
        turn_speed_ = turn_speed;
    }

    void set_drive_speed(float drive_speed)
    {
        drive_speed_ = drive_speed;
    }

    void set_target_distance(float target_distance)
    {
        target_distance_ = target_distance;
    }

    float get_scan_range()
    {
        return scan_range_;
    }

    float get_correction_threshold()
    {
        return correction_threshold_;
    }

    float get_turn_speed()
    {
        return turn_speed_;
    }

    float get_drive_speed()
    {
        return drive_speed_;
    }

    float get_target_distance()
    {
        return target_distance_;
    }

    void reconfigure(float scan_range, float correction_threshold, float turn_speed, float drive_speed, float target_distance);

    void button_input(const std_msgs::UInt8 &msg);

    CorrectionReport correct_angle();

    bool turn(float radians);
    float drive(float distance);
};

void sayHello();

Target get_target(sensor_msgs::LaserScanConstPtr scan);

std::vector<float> get_correction(std::vector<Point> &points, int first_edge_index, int second_edge_index);
