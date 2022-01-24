#include <unistd.h>
#include <string>
#include <fstream>
#include <experimental/filesystem>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"
#include "robot_main/Routine.h"

namespace fs = std::experimental::filesystem;

robot_main::Routine routine;
bool execute_routine = false;

void reconfigure(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd)
{
    rd.set_correction_threshold(config.correction_threshold);
}

void set_routine_callback(const robot_main::RoutineConstPtr &msg)
{
    ROS_INFO("Received routine: %i steps of %fm", msg->nb_steps, msg->step_distance);
    
    routine.nb_steps = msg->nb_steps;
    routine.step_distance = msg->step_distance;
    routine.freq_start = msg->freq_start;
    routine.freq_end = msg->freq_end;
    routine.power_level = msg->power_level;
    routine.time = msg->time;
    routine.sweep = msg->sweep;
}

void start_routine_callback(const std_msgs::Bool &msg)
{
    if (msg.data)
        ROS_INFO("Starting routine");

    execute_routine = msg.data;
}

void button_callback(const std_msgs::UInt8 &msg)
{
    ROS_INFO("Routine: %i steps of %fm ; Execute: %s", routine.nb_steps, routine.step_distance, execute_routine ? "yes" : "no");
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
    // ros::Subscriber sub_buttons = nh.subscribe("/buttons", 1, &RobotDriver::button_input, &rd);
    ros::Subscriber sub_buttons = nh.subscribe("/buttons", 1, &button_callback);
    ros::Subscriber sub_set_routine = nh.subscribe("/set_routine", 1, &set_routine_callback);
    ros::Subscriber sub_start_routine = nh.subscribe("/start_routine", 1, &start_routine_callback);

    // Publisher
    ros::Publisher pub_routine_progress = nh.advertise<std_msgs::UInt8>("/routine_progress", 1);

    // Dynamic reconfigure
    dynamic_reconfigure::Server<robot_main::GlobalConfig> server;
    dynamic_reconfigure::Server<robot_main::GlobalConfig>::CallbackType f;

    f = boost::bind(&reconfigure, _1, _2, boost::ref(rd));
    server.setCallback(f);

    // Report
    std::ofstream report_file("/home/husarion/robot_reports/report.csv", std::ofstream::trunc); 
    report_file << "angle,laser,encoders" << std::endl;

    // Main loop
    int current_step = 0;
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (execute_routine && current_step < routine.nb_steps)
        {
            ROS_INFO("RobotMain: === Routine step %i ===", current_step + 1);

            float current_angle = rd.correct_angle();

            Distance dist = rd.drive(routine.step_distance);

            report_file << current_angle << "," << dist.laser << "," << dist.encoders << std::endl;

            ROS_INFO("RobotMain: step %i done", current_step + 1);

            current_step ++;
            
            auto msg = std_msgs::UInt8();
            msg.data = current_step;
            pub_routine_progress.publish(msg);

            sleep(1);
        }
        else if (execute_routine && current_step == routine.nb_steps)
        {
            execute_routine = false;
            current_step = 0;

            report_file.close();
            sleep(2);
            report_file = std::ofstream("/home/husarion/robot_reports/report.csv", std::ofstream::trunc); 
            report_file << "angle,laser,encoders" << std::endl;
        }
        else if (!execute_routine && current_step != 0)
        {
            // In case of stop from GUI, TODO: be able choose between "pause" or "stop" the routine
            current_step = 0;
        }
    }

    return 0;
}
