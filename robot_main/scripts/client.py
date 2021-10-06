#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("robot_main_node", timeout=30)

    config = {
        "drive_speed": 0.1,
    }

    client.update_configuration(config)