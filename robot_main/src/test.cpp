/*
 * Correction Error Test
 */

#include <stdlib.h>
#include <robot_main/GlobalConfig.h>
#include "robot_driver/robot_driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_main_test_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    RobotDriver rd(nh);
    rd.reconfigure(2.9, 0.005, 0.005, 0.15, 0);
    
    std::string file_name = "/home/husarion/ros_workspace/data.csv";
    std::ofstream log_file;
    log_file.open(file_name);

    log_file << "id,rotation,final" << std::endl;
    
    int nb_tests = 5;
    for (int test = 0; test < nb_tests; test++)
    {
        float r = rand();
        float angle = 0.02 + r / (RAND_MAX / 0.1);

        rd.turn(rand() > (RAND_MAX / 2), angle);

        CorrectionReport report = rd.correct_angle();

        log_file << test << "," << angle << "," << report.polyfit[1] << std::endl;

        std::cout << "[";
        int pos = test * 70 / nb_tests;
        for (int i = 0; i < 70; ++i)
        {
            if (i < pos) std::cout << "=";
            else if (i == pos) std::cout << ">";
            else std::cout << " ";
        }
        std::cout << "]\r";
        std::cout.flush();
    }

    std::cout << "Done" << std::endl;

    log_file.close();

    return 0;
}
