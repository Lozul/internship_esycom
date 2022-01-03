#include "robot_driver/robot_driver.h"

RobotDriver::RobotDriver(ros::NodeHandle nh)
{
    nh_ = nh;

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Waiting for get_correction service");
    bool srv_ok = ros::service::waitForService("get_correction");

    if (!srv_ok)
    {
        ROS_ERROR("get_correction timed out");
        throw std::runtime_error("timeout");
    }

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

bool compare_float(float a, float b, float epsilon = 0.05)
{
    float diff = a - b;

    return ((diff < epsilon) && (-diff < epsilon));
}

std::pair<int, int> find_borders(const std::vector<Point> &points, float target_distance, float theta_angle, float epsilon = 0.1)
{
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

Target get_target(sensor_msgs::LaserScanConstPtr scan)
{
    Target target;

    // Filter laser data
    ROS_DEBUG("get_target: filtering laser data...");
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

        // Save valid point TODO: doc offset 0.0246
        Point p;
        p.range = range;
        p.angle = angle - 0.0246;
        points.push_back(p);
    }
    ROS_DEBUG("get_target: searching through %lu points...", points.size());

    if (points.size() == 0)
    {
        ROS_ERROR("get_target: found 0 valid point in laser scan");
        throw (NO_SCAN_POINT);
    }

    // Estimate target distance
    float first_range = points[0].range;
    float last_range = points[points.size() - 1].range;

    if (!compare_float(first_range, last_range, 0.02))
        ROS_WARN("get_target: difference between front ranges: %.3f %.3f", first_range, last_range);

    float target_distance = (first_range + last_range) / 2;
    ROS_DEBUG("get_target: estimated target distance is %.3f", target_distance);

    // Theta angle
    float theta_angle = M_PI - atan(0.5 / target_distance);
    ROS_DEBUG("get_target: theta angle = %.3f", theta_angle);

    target.range = target_distance;
    target.theta_angle = theta_angle;

    // Accuracy
    float accuracy = 1;

    if (3 < target_distance && target_distance <= 5)
        accuracy = 2;
    else if (5 < target_distance)
        accuracy = 2.5;

    // Searching target borders
    auto borders = find_borders(points, target_distance, theta_angle, accuracy * target_distance / 100);

    if (borders.first >= points.size() || borders.second >= points.size())
    {
        ROS_ERROR("RobotDriver: find_borders failed, out of bounds index (%i or %i)", borders.first, borders.second);
        throw (FIND_BORDERS_FAILED);
    }

    // Report in case of error
    if (borders.first == -1 || borders.second == -1)
    {
        ROS_ERROR("RobotDriver: failed to find target (left=%i, right=%i)", borders.first, borders.second);
        throw (NO_BORDERS);
    }

    target.first_edge_index = borders.first;
    target.second_edge_index = borders.second;

    target.points = points;

    return target;
}

std::vector<float> get_correction(std::vector<Point> &points, int first_edge_index, int second_edge_index)
{
    std::vector<float> xValues;
    std::vector<float> yValues;

    for (auto p : points)
    {
        if (points[first_edge_index].angle < p.angle && p.angle < points[second_edge_index].angle)
            continue;

        // Polar to cartesian
        float x = p.range * std::sin(M_PI - p.angle);
        float y = p.range * std::cos(M_PI - p.angle);
        xValues.push_back(x);
        yValues.push_back(y);
    }

    std::vector<float> fit = polyfit_boost(xValues, yValues, 1);
    fit[1] = std::atan(fit[1]);

    return fit;
}

CorrectionReport RobotDriver::correct_angle()
{
    // Init report
    CorrectionReport report;
    report.success = false;

    float correction = 1;
    float total_correction = 0;

    ROS_INFO("Waiting for get_correction service");
    ros::service::waitForService("get_correction");

    robot_driver::GetCorrection srv;

    do 
    {
        if (!ros::service::call("get_correction", srv))
        {
            ROS_ERROR("Failed to call service get_correction");
            return report;
        }
        else if (!srv.response.success)
        {
            ROS_ERROR("get_correction service failed");
            return report;
        }

        correction = srv.response.angle;

        if (std::abs(correction) >= correction_threshold_)
        {
            turn(correction < 0, std::abs(correction));
            total_correction += correction;
        }
    } while (std::abs(correction) >= correction_threshold_);

    ROS_INFO("RobotDriver: correction made or no correction to be made (%.3f rad)", total_correction);

    report.success = true;
    return report;
}

bool RobotDriver::turn(bool clockwise, float radians)
{
    while (radians < 0) radians += 2 * M_PI;
    while (radians > 2*M_PI) radians -= 2 * M_PI;
    ROS_DEBUG("RobotDriver: turning %.3f", radians);

    // We will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    // Record the starting transform from the odometry to the base link
    listener_.lookupTransform("base_link", "odom",
        ros::Time(0), start_transform);

    // We will be sending commands of type "twist"
    geometry_msgs::Twist move;

    // The command will be to turn at turn_speed_ rad/s
    float turn_speed = 0.005;
    move.linear.x = move.linear.y = 0;
    move.angular.z = clockwise ? -turn_speed : turn_speed;

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
    // Drive settings
    float xf = distance * error_margin_;

    float cruise_speed = std::min(xf / 2, max_speed_);
    float T = 1.5 * (xf / cruise_speed);

    float frequency = 100;
    float time_step = 1.0 / frequency;

    float t = 0;
    bool done = false;

    // ROS
    geometry_msgs::Twist move;
    ros::Rate rate(frequency);

    while (!done && nh_.ok())
    {
        // Determining speed
        if (0 <= t && t < T / 3)
            move.linear.x = (3 * cruise_speed * t) / T;
        else if (T / 3 <= t && t < 2 * T / 3)
            move.linear.x = cruise_speed;
        else if (2 * T / 3 <= t <= T)
            move.linear.x = 3 * cruise_speed * (1 - t / T);

        pub_cmd_vel_.publish(move);

        // Increasing time counter
        t += time_step;

        // Pause
        rate.sleep();

        // If total time is elapsed, finish driving
        done = t > T;
    }

    // To be sure that robot is stopped
    move.linear.x = 0;
    pub_cmd_vel_.publish(move);

    // TODO: return traveled distance (using laser?)
    return 0;
}

void sayHello()
{
    ROS_INFO("Coucou");
}
