#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"

void callback(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd)
{
    rd.reconfigure(config.scan_range, config.correction_threshold, config.turn_speed, config.drive_speed);
}

int main(int argc, char **argv)
{
    // RobotDriver parameters
    float scan_range;
    float correction_threshold;
    float turn_speed;
    float drive_speed;

    // ROS init
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    sayHello();
    
    // Get parameters
    nh.param<float>("/robot_driver/scan_range", scan_range, 3.0);
    nh.param<float>("/robot_driver/correction_threshold", correction_threshold, 0.1);
    nh.param<float>("/robot_driver/turn_speed", turn_speed, 0.5);
    nh.param<float>("/robot_driver/drive_speed", drive_speed, 0.25);

    // RobotDriver init
    RobotDriver rd(nh, scan_range, correction_threshold, turn_speed, drive_speed);

    ros::Subscriber sub = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);

    dynamic_reconfigure::Server<robot_main::GlobalConfig> server;
    dynamic_reconfigure::Server<robot_main::GlobalConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2, boost::ref(rd));
    server.setCallback(f);

    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
