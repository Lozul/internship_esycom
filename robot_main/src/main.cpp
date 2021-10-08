#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"
#include "robot_main/routine.h"

bool execute_routine = false;

void reconfigure(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd, Routine &routine)
{
    rd.reconfigure(config.scan_range, config.correction_threshold, config.turn_speed, config.drive_speed);
    routine.reconfigure(config.steps, config.step_distance);
}

void routine_callback(const std_msgs::Bool::ConstPtr &msg)
{
    execute_routine = msg->data;

    if (msg->data)
        ROS_INFO("RobotMain: routine started");
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

    // Routine init
    Routine routine;

    // Subscribers
    ros::Subscriber sub_buttons = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);
    ros::Subscriber sub_routine = nh.subscribe("/routine", 1, &routine_callback);

    // Dynamic reconfigure
    dynamic_reconfigure::Server<robot_main::GlobalConfig> server;
    dynamic_reconfigure::Server<robot_main::GlobalConfig>::CallbackType f;

    f = boost::bind(&reconfigure, _1, _2, boost::ref(rd), boost::ref(routine));
    server.setCallback(f);

    // Main loop
    int current_step = 0;
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (execute_routine && current_step < routine.steps)
        {
            ROS_INFO("RobotMain: routine step %i, angle correction...", current_step + 1);
            rd.correct_angle();
            ROS_INFO("RobotMain: routine step %i, driving to next stop", current_step + 1);
            rd.drive(routine.step_distance);
            ROS_INFO("RobotMain: routine step %i done", current_step + 1);
            current_step ++;
        }
        else if (current_step == routine.steps)
        {
            execute_routine = false;
            current_step = 0;
        }
    }

    return 0;
}
