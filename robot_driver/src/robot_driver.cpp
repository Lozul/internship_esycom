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

bool RobotDriver::correct_angle()
{
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
            return 0;
        }
        else if (!srv.response.success)
        {
            ROS_ERROR("get_correction service failed");
            return 0;
        }

        correction = srv.response.angle;

        if (std::abs(correction) >= correction_threshold_)
        {
            turn(correction < 0, std::abs(correction));
            total_correction += correction;
        }
    } while (std::abs(correction) >= correction_threshold_);

    ROS_INFO("RobotDriver: correction made or no correction to be made (%.3f rad)", total_correction);

    return correction;
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

    // The speed will increase to cruise_speed halfway then decrease for smooth movement
    float cruise_speed = radians / 2;
    move.linear.x = move.linear.y = move.angular.z = 0;

    // The axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0, 0, clockwise ? 1 : -1);

    ros::Rate rate(100);
    bool done = false;
    double angle_turned = 0.0;
    while (angle_turned < radians && !done && nh_.ok())
    {
        // Determine speed
        angle_turned = fabs(angle_turned);
        move.angular.z = 2 * cruise_speed * (1e-2 / radians); // Default speed for startup

        // The "magic numbers" where found by experience
        //  bellow 1e-2 we will apply the startup speed,
        //  then we will increase until halfway through, then decrease
        if (1e-2 < angle_turned && angle_turned < radians / 2)
            move.angular.z = 2 * cruise_speed * (angle_turned / radians);
        else if (radians / 2 <= angle_turned && angle_turned < radians)
            move.angular.z = 2 * cruise_speed * (1 - angle_turned / radians);

        move.angular.z *= clockwise ? -1 : 1;

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

        // Compute turned angle
        tf::Transform relative_transform =
            start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis =
            relative_transform.getRotation().getAxis();

        angle_turned = relative_transform.getRotation().getAngle();

        if (fabs(angle_turned) < 1.0e-2)
            continue;

        if (actual_turn_axis.dot(desired_turn_axis) < 0)
            angle_turned = 2 * M_PI - angle_turned;

        if (angle_turned >= radians)
            done = true;
    }

    ROS_INFO("Angle turned: %f", angle_turned);

    move.angular.z = 0;
    pub_cmd_vel_.publish(move);

    return done;
}

Distance RobotDriver::drive(float distance)
{
    // Record starting position
    Distance start = getTargetDistance();

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

    // Record final position
    Distance res = getTargetDistance();

    // Return traveled distance
    res.encoders -= start.encoders;
    res.laser -= start.laser;

    return res;
}

Distance getTargetDistance()
{
    Distance res;

    auto msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose");
    res.encoders = msg.pose.point.x;

    while (true)
    {
        auto msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
        int len = sizeof(msg.ranges) / sizeof(msg.ranges[0]);

        float a = msg.ranges[0];
        float b = msg.ranges[len - 1];

        if ((a < msg.range_min || a > msg.range_max)
                || (b < msg.range_min || b > msg.range_max))
            continue;

        res.laser = (a + b) / 2;
        break;
    }

    return res;
}

void sayHello()
{
    ROS_INFO("Coucou");
}
