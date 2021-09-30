#include "robot_driver/robot_driver.h"

const float pi = 3.14;

RobotDriver::RobotDriver(ros::NodeHandle nh, float scan_range, float correction_threshold, float turn_speed)
{
    if (scan_range < 0)
        scan_range = -scan_range;

    nh_ = nh;
    scan_range_ = scan_range;
    correction_threshold_ = correction_threshold;
    turn_speed_ = turn_speed;

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void RobotDriver::button_input(const std_msgs::UInt8 &msg)
{
    if (msg.data == 1)
        correct_angle();
    else
        ros::shutdown();
}

Wall RobotDriver::get_wall_()
{
    Wall wall;
    
    sensor_msgs::LaserScanConstPtr scan =
        ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");

    if (!scan)
    {
        ROS_ERROR("RobotDriver: No scan message");
    }

    wall.range = scan->ranges[0];
    wall.angle = scan->angle_min;

    for (int i = 1; i < scan->ranges.size(); i++)
    {
        float range = scan->ranges[i];

        if (range < scan->range_min || range > scan->range_max)
            continue;

        float angle = scan->angle_min + scan->angle_increment * i;

        if (angle > -scan_range_ && angle < scan_range_)
            continue;

        if (wall.range > range)
        {
            wall.range = range;
            wall.angle = angle;
        }
    }

    return wall;
}

void RobotDriver::correct_angle()
{
    bool done = false;

    while (!done && nh_.ok())
    {
        Wall wall = get_wall_();
        ROS_DEBUG("RobotDriver: wall hit at %.3f [r] and %.3f [m]", wall.angle, wall.range);

        float correction = (pi - std::abs(wall.angle)) * (wall.angle > 0 ? -1 : 1);

        if (correction < correction_threshold_ && -correction < correction_threshold_)
        {
            ROS_INFO("RobotDriver: correction made or no correction to be made");
            done = true;
            break;
        }

        ROS_DEBUG("RobotDriver: angle to correction: %.3f [r]", correction);

        if (turn(wall.angle > 0, correction))
            ROS_DEBUG("RobotDriver: step correction made");
        else
            ROS_DEBUG("RobotDriver: step correction not completed");
    }
}

bool RobotDriver::turn(bool clockwise, float radians)
{
    // Wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom",
        ros::Time(0), ros::Duration(1.0));

    // We will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    // Record the starting transfrom from the laser to the base link
    listener_.lookupTransform("base_link", "odom",
        ros::Time(0), start_transform);

    // We will be sending commands of type "twist"
    geometry_msgs::Twist move;

    // The command will be to turn at 0.5 rad/s
    move.linear.x = move.linear.y = 0;
    move.angular.z = clockwise ? -turn_speed_ : turn_speed_;

    // The axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0, 0, clockwise ? 1 : -1);

    ros::Rate rate(10);
    bool done = false;
    while (!done && nh_.ok())
    {
        // Send the drive command
        pub_cmd_vel_.publish(move);
        rate.sleep();

        // Get the current transform
        try
        {
            listener_.lookupTransform("base_link", "odom",
                ros::Time(0), current_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            break;
        }

        tf::Transform relative_transform =
            start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis =
            relative_transform.getRotation().getAxis();

        double angle_turned = relative_transform.getRotation().getAngle();

        if (fabs(angle_turned) < 1.0e-2)
            continue;

        if (actual_turn_axis.dot(desired_turn_axis) < 0)
            angle_turned = 2 * pi - angle_turned;

        if (angle_turned > radians)
            done = true;
    }

    move.angular.z = 0;
    pub_cmd_vel_.publish(move);

    return done;
}

void sayHello()
{
    ROS_INFO("Coucou");
}