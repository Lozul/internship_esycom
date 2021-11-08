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

std::pair<int, int> find_borders(const std::vector<Point> &points, float target_distance, float theta_angle)
{
    float epsilon = 0.1;

    bool valid[points.size()] = { 0 };

    for (int i = 0; i < points.size(); i++)
    {
        float angle = points[i].angle;

        if (angle > -theta_angle && angle < theta_angle)
            continue;

        float cos_angle = std::cos(points[i].angle);
        float range_min = std::abs((target_distance - epsilon) / cos_angle);
        float range_max = std::abs((target_distance + epsilon) / cos_angle);

        valid[i] = range_min <= points[i].range && points[i].range <= range_max;

        // ROS_DEBUG("RobotDriver: %.3f <= %.3f (%.3f) <= %.3f", range_min, points[i].range, points[i].angle, range_max);
    }

    std::vector<std::pair<int, int>> indexes;
    std::vector<int> lengths;
    int start = 0;
    int length = 0;
    bool streak = false;

    for (int i = 0; i < points.size(); i++)
    {
        if (valid[i] && !streak)
        {
            start = i;
            length += 1;
            streak = true;
        }
        else if (valid[i] && streak)
        {
            length += 1;
        }
        else if (!valid[i] && streak)
        {
            indexes.push_back(std::make_pair(start, start + length - 1));
            lengths.push_back(length);
            length = 0;
            streak = false;
        }
    }

    if (streak)
    {
        indexes.push_back(std::make_pair(start, start + length - 1));
        lengths.push_back(length);
    }

    if (indexes.size() == 0)
        return std::make_pair(-1, -1);

    if (indexes.front().first == 0 && indexes.back().second == points.size() - 1)
    {

        indexes.front().first = indexes.front().second;
        indexes.front().second = indexes.back().first;
        lengths[0] += lengths.back();

        indexes.pop_back();
        lengths.pop_back();
    }

    int mx_i = 0;

    for (int i = 0; i < indexes.size(); i++)
    {
        if (lengths[i] > lengths[mx_i])
            mx_i = i;
    }

    auto result = indexes[mx_i];

    return result;
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

    report.last_scan = scan;

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

        if (angle > -(M_PI / 2) && angle < (M_PI / 2))
            continue;

        // Save valid point
        Point p;
        p.range = range;
        p.angle = angle;
        points.push_back(p);
    }
    ROS_INFO("RobotDriver: searching through %lu points...", points.size());

    if (points.size() == 0)
    {
        ROS_ERROR("RobotDriver: no valide point found");
        return report;
    }

    // Estimate target distance
    float first_range = points[0].range;
    float last_range = points[points.size() - 1].range;

    if (!compare_float(first_range, last_range, 0.02))
        ROS_WARN("RobotDriver: big difference between front ranges: %.3f %.3f", first_range, last_range);

    float target_distance = (first_range + last_range) / 2;
    ROS_DEBUG("RobotDriver: estimated target distance is %.3f", target_distance);

    float theta_angle = M_PI - atan(0.5 / target_distance);
    ROS_DEBUG("RobotDriver: theta angle = %.3f", theta_angle);

    report.target_distance = target_distance;
    report.theta_angle = theta_angle;

    // Searching target borders
    auto borders = find_borders(points, target_distance, theta_angle);

    if (borders.first >= points.size() || borders.second >= points.size())
    {
        ROS_ERROR("RobotDriver: find_borders failed, out of bounds index (%i or %i)", borders.first, borders.second);
        return report;
    }

    // Update report borders if valid
    if (borders.first != -1)
        report.first = points[borders.first];
    if (borders.second != -1)
        report.second = points[borders.second];

    // Report in case of error
    if (borders.first == -1 || borders.second == -1)
    {
        ROS_ERROR("RobotDriver: failed to find target (left=%i, right=%i)", borders.first, borders.second);
        return report;
    }
    
    /*
     * Here we transpose the angles between 0 and 2 pi
     * Originaly they are between -pi and pi and I kept getting lost in my calculations so "flute" as we say in french
     */
    float first = points[borders.first].angle;
    float second = points[borders.second].angle;

    if (first < 0)
        first += 2 * M_PI;
    if (second < 0)
        second += 2 * M_PI;

    float left = first > second ? first : second;
    float right = first > second ? second : first;
    ROS_INFO("RobotDriver: Target located between %.3fr and %.3fr at %.3fm", left, right, target_distance);

    // Searching correction angle
    Point target;
    float target_diff = 999;
    bool found = false;
    for (auto p : points)
    {
        float a = p.angle;

        if (a < 0)
            a += 2 * M_PI;

        if (a < right || a > left)
            continue;

        float current_diff = std::abs(target_distance - p.range);

        // ROS_DEBUG("RobotDriver: (a=%.3f, r=%.3f) diff with estimated target is %.3f", a, p.range, current_diff);

        if (compare_float(p.range, target_distance, 0.1) && current_diff < target_diff)
        {
            ROS_DEBUG("RobotDriver: target updated");
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

    ROS_INFO("RobotDriver: Correction point found (%.3f rad, %.3f m)", target.angle, target.range);

    report.correction_point = target;

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

    // Record the starting transform from the odometry to the base link
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

float RobotDriver::drive(float distance)
{
    // We will record position here
    geometry_msgs::Point start_position;
    geometry_msgs::Point current_position;

    // Record the starting position
    auto ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");

    if (!ptr)
    {
        ROS_ERROR("RobotDriver: failed to record starting position");
        return 0;
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
            return 0;
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

    return dist_moved;
}

void sayHello()
{
    ROS_INFO("Coucou");
}
