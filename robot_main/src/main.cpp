#include <unistd.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"
#include "robot_main/Routine.h"
#include "robot_main/data_exporter.h"
#include "LMShid.h"

robot_main::Routine routine;
bool execute_routine = false;

void reconfigure(robot_main::GlobalConfig &config, uint32_t level, RobotDriver &rd)
{
    rd.reconfigure(config.scan_range, config.correction_threshold, config.turn_speed, config.drive_speed, config.target_distance);
}

void set_routine_callback(const robot_main::RoutineConstPtr &msg)
{
    ROS_INFO("Received routine: %i steps of %fm", msg->nb_steps, msg->step_distance);
    
    routine.nb_steps = msg->nb_steps;
    routine.step_distance = msg->step_distance;
    routine.freq = msg->freq;
}

void start_routine_callback(const std_msgs::Bool &msg)
{
    execute_routine = msg.data;
}

void button_callback(const std_msgs::UInt8 &msg)
{
    ROS_INFO("Routine: %i steps of %fm ; Execute: %s", routine.nb_steps, routine.step_distance, execute_routine ? "yes" : "no");
}

int main(int argc, char **argv)
{
    fnLMS_Init();
    fnLMS_SetTestMode(1);

    // ROS init
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    sayHello();

    // RobotDriver init
    RobotDriver rd(nh);

    // Routine init
    routine.nb_steps = 0;
    routine.step_distance = 0.0;
    routine.freq = 0;

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

    std::string report_folder = "/home/husarion/ros_workspace/reports/";
    std::string after_correction_suffix = "_after";
    std::string extension = ".csv";

    // LMS device
    int nb_plugged = fnLMS_GetNumDevices();

    if (nb_plugged == 0)
    {
        ROS_ERROR("RobotMain: no Vaunix LMS devices located.");
        return 1;
    }

    DEVID generator_id;
    DEVID active_devices[MAXDEVICES];

    fnLMS_GetDevInfo(active_devices);
    generator_id = active_devices[0];

    int status = fnLMS_InitDevice(generator_id);
    ROS_INFO("RobotMain: generator status: %s", fnLMS_perror(status));

    if (status & ERROR_STATUS_MASK) return 1;

    // Main loop
    int current_step = 0;
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (execute_routine && current_step < routine.nb_steps)
        {
            // ROS_DEBUG("RobotMain: routine step %i, angle correction...", current_step + 1);

            // CorrectionReport report = rd.correct_angle();

            // Export report
            // std::string file_name = report_folder + std::to_string(current_step) + extension;
            // export_correction_report(report, file_name);

            // Export laser scan after correction
            // file_name = report_folder + std::to_string(current_step) + after_correction_suffix + extension;
            // auto scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
            // export_laser_scan(scan, file_name);

            ROS_INFO("RobotMain: routine step %i, emit frequency %i", current_step + 1, routine.freq);

            // TODO

            ROS_DEBUG("RobotMain: routine step %i, driving to next stop", current_step + 1);

            // rd.drive(routine.step_distance);

            ROS_INFO("RobotMain: routine step %i done", current_step + 1);

            current_step ++;
            
            auto msg = std_msgs::UInt8();
            msg.data = current_step;
            pub_routine_progress.publish(msg);

            sleep(1);
        }
        else if (current_step == routine.nb_steps)
        {
            execute_routine = false;
            current_step = 0;
        }
        else if (!execute_routine && current_step != 0)
        {
            // In case of stop from GUI, TODO: be able choose between "pause" or "stop" the routine
            current_step = 0;
        }
    }

    return 0;
}
