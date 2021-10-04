#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <robot_driver/DriverConfig.h>
#include "robot_driver/robot_driver.h"

void callback(robot_driver::DriverConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure request:\n"
        "\tscan_range: %.3f\n"
        "\tcorrection_threshold: %.2f\n"
        "\tturn_speed: %.3f",
        config.scan_range, config.correction_threshold, config.turn_speed);
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

    // Dynamic reconfigure
    dynamic_reconfigure::Server<robot_driver::DriverConfig> server;
    dynamic_reconfigure::Server<robot_driver::DriverConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber sub = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);

    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
