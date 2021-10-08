#include "robot_driver/robot_driver.h"

const float pi = 3.14;

RobotDriver::RobotDriver(ros::NodeHandle nh)
    : RobotDriver(nh, 0, 0, 0, 0)
{}

RobotDriver::RobotDriver(ros::NodeHandle nh, float scan_range, float correction_threshold, float turn_speed, float drive_speed)
{
    if (scan_range < 0)
        scan_range = -scan_range;

    nh_ = nh;
    scan_range_ = scan_range;
    correction_threshold_ = correction_threshold;
    turn_speed_ = turn_speed;
    drive_speed_ = drive_speed;

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Wait for the listener to get the first message
    ros::Duration timeout(10.0);
    bool tf_ok = listener_.waitForTransform("base_link", "odom",
        ros::Time(0), timeout);

    if (!tf_ok)
    {
        ROS_ERROR("RobotDriver: TransformListener timed out after %i", timeout.sec);
        throw std::runtime_error("timeout");
    }
}

void RobotDriver::button_input(const std_msgs::UInt8 &msg)
{
    if (msg.data == 1)
        correct_angle();
    else
        drive(1.0);
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

void RobotDriver::reconfigure(float scan_range, float correction_threshold, float turn_speed, float drive_speed)
{
    ROS_DEBUG("RobotDriver: reconfigure request:\n"
        "\tscan_range: %.3f\n"
        "\tcorrection_threshold: %.2f\n"
        "\tturn_speed: %.3f\n"
        "\tdrive_speed: %.3f",
        scan_range, correction_threshold, turn_speed, drive_speed);

    scan_range_ = scan_range;
    correction_threshold_ = correction_threshold;
    turn_speed_ = turn_speed;
    drive_speed_ = drive_speed;
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
            ROS_INFO("RobotDriver: correction made or no correction to be made (%.3f r)", correction);
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
    // We will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    // Record the starting transfrom from the odometry to the base link
    listener_.lookupTransform("base_link", "odom",
        ros::Time(0), start_transform);

    // We will be sending commands of type "twist"
    geometry_msgs::Twist move;

    // The command will be to turn at turn_speed_ rad/s
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

bool RobotDriver::drive(float distance)
{
    // We will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    // Record the starting transform from the odometry to the base link
    listener_.lookupTransform("base_link", "odom",
        ros::Time(0), start_transform);

    // We will be sending commands of type "twist"
    geometry_msgs::Twist move;
    // The command will be to go forward at drive_speed_ m/s
    move.linear.y = move.angular.z = 0;
    move.linear.x = drive_speed_;

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

        // See how far we've reached
        tf::Transform relative_transform =
            start_transform.inverse() * current_transform;
        float dist_moved = relative_transform.getOrigin().length();

        if (dist_moved > distance) done = true;
    }

    move.linear.x = 0;
    pub_cmd_vel_.publish(move);

    return done;
}

void sayHello()
{
    ROS_INFO("Coucou");
}
