#include <stdlib.h>
#include "robot_driver/robot_driver.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_main_test_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    RobotDriver rd(nh);

    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        rd.turn(false, 0.7);
        break;
    }

    return 0;
}
