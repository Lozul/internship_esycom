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
    // ROS init
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    sayHello();

    // RobotDriver init
    RobotDriver rd(nh);

    // Subscribers
    ros::Subscriber sub = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);

    // Dynamic reconfigure
    dynamic_reconfigure::Server<robot_main::GlobalConfig> server;
    dynamic_reconfigure::Server<robot_main::GlobalConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2, boost::ref(rd));
    server.setCallback(f);

    // Main loop
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
