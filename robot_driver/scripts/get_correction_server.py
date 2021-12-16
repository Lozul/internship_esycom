#!/usr/bin/env python
from math import cos, sin, atan

from robot_driver.srv import GetCorrection, GetCorrectionResponse
from sensor_msgs.msg import LaserScan
import numpy as np
import rospy


def polar_cartesian(p):
    a, r = p
    return r * sin(np.pi - a), r * cos(np.pi - a)


def hand_get_correction(req):
    resp = GetCorrectionResponse(False, 0)

    rospy.loginfo("Received a request for correction")

    # Get laser scan
    laser_scan = rospy.wait_for_message('/scan', LaserScan)

    # Associate ranges with angles
    angles = [laser_scan.angle_min + laser_scan.angle_increment * i for i in range(len(laser_scan.ranges))]
    scan = np.array(list(zip(angles, laser_scan.ranges)))

    # Filter scan
    ## Keep points in valid range
    mask = [laser_scan.range_min <= r <= laser_scan.range_max for r in scan[:, 1]]
    points = scan[mask]

    if len(points) == 0:
        rospy.logerror("No points in range")
        return resp

    ## Remove laser angle error
    points = np.apply_along_axis(lambda p: (p[0] - 0.0246, p[1]), 1, points)

    ## Keep front points only
    mask = abs(points[:, 0]) > np.pi / 2
    points = points[mask]

    if len(points) == 0:
        rospy.logerror("No points in front")
        return resp

    ## Extract target points
    ### Estimate target distance
    first_range, last_range = valid_points[0, 1], valid_points[-1, 1]

    if abs(first_range - last_range) > 0.01:
        rospy.logerror("Difference between first and last range of scan: %f and %f", first_range, last_range)
        return resp

    target_distance = (first_range + last_range) / 2

    ### Angle range to scan based on target_distance
    theta_angle = np.pi - atan(0.5 / target_distance)

    ### Keep valid target points
    accuracy = 1

    if 3 < target_distance <= 5:
        accuracy = 2
    elif 5 <= target_distance:
        accuracy = 2.5

    epsilon = accuracy * target_distance / 100

    def is_valid(p):
        a, r = p

        if abs(a) < theta_angle: return False

        cos_a = cos(a)
        min_r = abs((target_distance - epsilon) / cos_a)
        max_r = abs((target_distance + epsilon) / cos_a)

        return min_r <= r <= max_r
    
    mask = [is_valid(p) for p in points]
    points = points[mask]

    if len(points):
        rospy.logerror("No valid target points")
        return resp

    # Estimate target slope, therefore angle to correct
    points_cartesian = np.array([polar_cartesian(p) for p in points])
    fit = np.polyfit(points_cartesian[:, 0], points_cartesian[:, 1], 1)

    correction = atan(fit[0])

    # Return result
    resp.success = True
    resp.angle = correction
    return resp


def server():
    rospy.init_node('get_correction_server')

    s = rospy.Service('get_correction', GetCorrection, hand_get_correction)

    rospy.loginfo("Ready")
    rospy.spin()


if __name__ == "__main__":
    server()
