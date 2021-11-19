#include <chrono>
#include <ctime>
#include <cmath>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"
#include "robot_main/routine.h"
#include "robot_main/data_exporter.h"

bool execute_routine = false;
int nb_test = 5;

void reconfigure(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd, Routine &routine)
{
    rd.reconfigure(config.scan_range, config.correction_threshold, config.turn_speed, config.drive_speed, config.target_distance);
    routine.reconfigure(config.steps, config.step_distance);

    nb_test = config.nb_test;
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

    std::string file_path = "/home/husarion/ros_workspace/data.csv";

    std::ofstream log_file;
    log_file.open(file_path);

    log_file << "id,rotation,final" << std::endl;

    float current_slope = rd.correct_angle().polyfit[1];
    ROS_INFO("%i tests to do. Starting slope: %f", nb_test, current_slope);

    for (int test = 0; test < nb_test; test++)
    {
        auto start = std::chrono::system_clock::now();

        float r = std::rand();
        float angle = 0.02 + r / (RAND_MAX / 0.1);

        rd.turn(rand() > (RAND_MAX / 2), angle);

        CorrectionReport report = rd.correct_angle();

        log_file << test << "," << angle << "," << report.polyfit[1] << std::endl;
    
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        double seconds = elapsed_seconds.count() * (nb_test - test - 1);
        double s = fmod(seconds, 60.);
        double m = seconds / 60;
        double h = seconds / 3600; 

        ROS_INFO("[%i] Rotation: %f ; Final: %f", test, angle, report.polyfit[1]);
        ROS_INFO("Estimated remaining time %02.0f:%02.0f:%02.0f", h, m, s);
    }

    log_file.close();

    ROS_INFO("STOP");

    return 0;
}
