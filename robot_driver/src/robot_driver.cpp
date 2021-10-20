#include "robot_driver/robot_driver.h"

RobotDriver::RobotDriver(ros::NodeHandle nh)
    : RobotDriver(nh, 0, 0, 0, 0, 0)
{}

RobotDriver::RobotDriver(ros::NodeHandle nh, float scan_range, float correction_threshold, float turn_speed, float drive_speed, float target_distance)
{
    if (scan_range < 0)
        scan_range = -scan_range;

    nh_ = nh;
    scan_range_ = scan_range;
    correction_threshold_ = correction_threshold;
    turn_speed_ = turn_speed;
    drive_speed_ = drive_speed;
    target_distance_ = target_distance;

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
        turn(false, M_PI / 2);
}

void RobotDriver::reconfigure(float scan_range, float correction_threshold, float turn_speed, float drive_speed, float target_distance)
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
    target_distance_ = target_distance;
}

bool compare_float(float a, float b, float epsilon = 0.05)
{
    float diff = a - b;

    return ((diff < epsilon) && (-diff < epsilon));
}

int find_border_index(std::vector<Point> points, float target_distance, bool left)
{
    float epsilon = 0.1;
    int i = left ? 0 : points.size() - 1;

    while (true)
    {
        float cos_angle = std::cos(points[i].angle);
        float range_min = std::abs((target_distance - epsilon) / cos_angle);
        float range_max = std::abs((target_distance + epsilon) / cos_angle);

        ROS_DEBUG("RobotDriver: %.3f <= %.3f (%.3f) <= %.3f", range_min, points[i].range, points[i].angle, range_max);

        if (points[i].range < range_min || points[i].range > range_max)
        {
            ROS_DEBUG("RobotDriver: Invalide");
            i += left ? -1 : 1;
            break;
        }
        else
        {
            i += left ? 1 : -1;
        }
    }

    if ((left && i == -1) || (!left && i == points.size()))
        return -1;

    return i;
}

CorrectionReport RobotDriver::correct_angle()
{
    // Init report
    CorrectionReport report;
    report.success = false;

    // Get laser data
    ROS_DEBUG("RobotDriver: getting laser data...");
    sensor_msgs::LaserScanConstPtr scan =
        ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");

    if (!scan)
    {
        ROS_ERROR("RobotDriver: no scan message");
        return report;
    }

    // Filter laser data
    ROS_DEBUG("RobotDriver: filtering laser data...");
    std::vector<Point> points;

    for (int i = 0; i < scan->ranges.size(); i++)
    {
        // Discard invalid values
        float range = scan->ranges[i];

        if (range < scan->range_min || range > scan->range_max)
            continue;

        // Discard values that are not in front of the robot
        float angle = scan->angle_min + scan->angle_increment * i;

        if (angle > -2.5 && angle < 2.5)
            continue;

        // Save valide point
        Point p;
        p.range = range;
        p.angle = angle;
        points.push_back(p);
    }
    ROS_DEBUG("RobotDriver: found %lu valide points", points.size());

    // Searching target borders
    int left_i = find_border_index(points, target_distance_, true);
    int right_i = find_border_index(points, target_distance_, false);

    if (left_i != -1)
        report.left = points[left_i];
    if (right_i != -1)
        report.right = points[right_i];

    if (left_i == -1 || right_i == -1)
    {
        ROS_ERROR("RobotDriver: failed to find target (left=%i, right=%i)", left_i, right_i);
        return report;
    }

    float left = points[left_i].angle;
    float right = points[right_i].angle;
    ROS_INFO("RobotDriver: target border l=%.3f and r=%.3f", left, right);

    if (left < 0)
        left += 2 * M_PI;
    if (right < 0)
        right += 2 * M_PI;

    // float e = (left - right) / 4;
    // ROS_INFO("RobotDriver: epsilon=%.3f", e);

    ROS_INFO("RobotDriver: target border l=%.3f and r=%.3f", left, right);

    // Searching correction angle
    Point target;
    float target_diff = 999;
    bool found = false;
    for (auto p : points)
    {
        float a = p.angle;

        if (a < 0)
            a += 2 * M_PI;

        if (a < right /*+ e*/ || a > left /*- e*/)
            continue;

        float current_diff = std::abs(target_distance_ - p.range);

        if (compare_float(p.range, target_distance_, 0.1) && current_diff < target_diff)
        {
            ROS_INFO("update target r=%.3f a=%.3f (%.3f)", p.range, p.angle, a);
            target.angle = p.angle;
            target.range = p.range;
            target_diff = current_diff;
            found = true;
        }
    }

    if (!found)
    {
        ROS_ERROR("RobotDriver: could not find correction point");
        return report;
    }

    ROS_INFO("RobotDriver: target found at a=%.3f r=%.3f", target.angle, target.range);

    float correction = M_PI - std::abs(target.angle);

    // Applying correction
    if (correction > correction_threshold_)
        turn(target.angle > 0, correction);

    ROS_INFO("RobotDriver: correction made or no correction to be made (%.3f rad)", correction);

    report.success = true;
    return report;
}

bool RobotDriver::turn(bool clockwise, float radians)
{
    while (radians < 0) radians += 2 * M_PI;
    while (radians > 2*M_PI) radians -= 2 * M_PI;

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

    ros::Rate rate(100);
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
            angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned > radians)
            done = true;
    }

    move.angular.z = 0;
    pub_cmd_vel_.publish(move);

    return done;
}

bool RobotDriver::drive(float distance)
{
    // We will record position here
    geometry_msgs::Point start_position;
    geometry_msgs::Point current_position;

    // Record the starting position
    auto ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");

    if (!ptr)
    {
        ROS_ERROR("RobotDriver: failed to record starting position");
        return false;
    }

    start_position = ptr->pose.position;

    // We will be sending commands of type `Twist`
    geometry_msgs::Twist move;
    move.linear.y = move.angular.z = 0;
    move.linear.x = drive_speed_;

    // Begin driving
    float dist_moved;
    ros::Rate rate(100);
    bool done = false;
    while (!done && nh_.ok())
    {
        pub_cmd_vel_.publish(move);
        rate.sleep();

        // Get the current position
        auto ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");

        if (!ptr)
        {
            ROS_ERROR("RobotDriver: failed to get current position");
            return false;
        }

        current_position = ptr->pose.position;
        dist_moved = current_position.x - start_position.x;

        if (dist_moved >= distance) done = true;
    }

    // Stop movement
    move.linear.x = 0;
    pub_cmd_vel_.publish(move);

    // Update target distance
    target_distance_ -= dist_moved;

    return done;
}

void sayHello()
{
    ROS_INFO("Coucou");
}
