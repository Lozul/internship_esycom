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
