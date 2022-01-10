\page GetCorrectionExplained How correction is determined

[TOC]

## Laser scan

First step of the algorithm is to record a laser scan of the room. This scan will be used to locate the target.

The invalid distances are then filtered out (using the metadata of the scan, see [ROS API](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)).

The reference frame of the laser is rotated by +0.0246 radians (value found by experiement). This error is taken into account by the algorithm.

## Extract target

Next step is to estimate the target distance. It should be in front of the robot, so the average distance of first (-pi) and the last (+pi) point are used.

The estimated distance is then used to determine the angle range where the target should be. The goal is to scan 1 m whatever the distance.

`theta = pi - atan(0.5 / target_distance)`

Only the angles between theta (respectively -theta) and pi (respectively -pi) are kept.

The points belonging to the target are filtered by checking the hypotenuse of the triangle they form with the robot and the center of the target.

![](valid_target_points.png)

## Estimate target slope

The target slope, therefore the angle to be correctedd, is obtained by performing a linear regression on the points of the target. Coefficient of degree one is used.

![](target_slope.png)
