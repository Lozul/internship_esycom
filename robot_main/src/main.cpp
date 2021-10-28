#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"
#include "robot_main/routine.h"

bool execute_routine = false;

void reconfigure(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd, Routine &routine)
{
    rd.reconfigure(config.scan_range, config.correction_threshold, config.turn_speed, config.drive_speed, config.target_distance);
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

    std::string report_folder = "/home/husarion/ros_workspace/reports/";
    std::string extension = ".csv";

    // Main loop
    int current_step = 0;
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (execute_routine && current_step < routine.steps)
        {
            ROS_DEBUG("RobotMain: routine step %i, angle correction...", current_step + 1);

            CorrectionReport report = rd.correct_angle();

            if (true)
            {
                // ROS_INFO("RobotMain: angle correction failed, saving laser data...");

                // Get current laser scan
                if (!report.last_scan)
                {
                    ROS_ERROR("RobotMain: correction report does not contains last scan");
                    continue;
                }

                auto scan = report.last_scan->get();

                // Open log file
                std::ofstream log_file;
                std::string file_name = report_folder + std::to_string(current_step) + extension;
                log_file.open(file_name);

                // Write entries of the target search algorithm
                log_file << report.target_distance << "," << report.theta_angle << std::endl;

                // Write target data if any
                if (report.first)
                {
                    Point first = report.first.value();
                    log_file << first.angle << "," << first.range << std::endl;
                }
                else
                    log_file << "0,0" << std::endl;

                if (report.second)
                {
                    Point second = report.second.value();
                    log_file << second.angle << "," << second.range << std::endl;
                }
                else
                    log_file << "0,0" << std::endl;

                if (report.correction_point)
                {
                    auto p = report.correction_point.value();
                    log_file << p.angle << "," << p.range << std::endl;
                }
                else
                    log_file << "0,0" << std::endl;

                // Write scan data
                for (int i = 0; i < scan->ranges.size(); i++)
                {
                    float range = scan->ranges[i];

                    if (range < scan->range_min || range > scan->range_max)
                        continue;

                    float angle = scan->angle_min + scan->angle_increment * i;

                    log_file << angle << "," << range << std::endl;
                }

                log_file.close();

                ROS_INFO("RobotMain: laser data saved at /home/husarion/ros_workspace/data.csv");

                execute_routine = false;
                continue;
            }

            ROS_DEBUG("RobotMain: routine step %i, driving to next stop", current_step + 1);

            rd.drive(routine.step_distance);

            ROS_DEBUG("RobotMain: routine step %i done", current_step + 1);

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
