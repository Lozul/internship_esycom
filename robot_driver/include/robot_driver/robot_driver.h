#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

struct Wall
{
    float range;
    float angle;
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

    Wall get_wall_();

public:
    RobotDriver(ros::NodeHandle nh, float scan_range, float correction_threshold, float turn_speed, float drive_speed);

    void button_input(const std_msgs::UInt8 &msg);

    void correct_angle();

    bool turn(bool clockwise, float radians);

    bool drive(float distance);
};

void sayHello();