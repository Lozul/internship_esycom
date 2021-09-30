#include <ros/ros.h>
#include "robot_driver/robot_driver.h"

int main(int argc, char **argv)
{
    float scan_range;
    float correction_threshold;
    float turn_speed;

    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    sayHello();

    nh.param<float>("/robot_driver/scan_range", scan_range, 3.0);
    nh.param<float>("/robot_driver/correction_threshold", correction_threshold, 0.1);
    nh.param<float>("/robot_driver/turn_speed", turn_speed, 0.5);

    RobotDriver rd(nh, scan_range, correction_threshold, turn_speed);

    ros::Subscriber sub = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);

    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}