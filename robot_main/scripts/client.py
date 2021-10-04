#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("robot_main_node", timeout=30)

    config = {
        "scan_range": 2.5,
        "correction_threshold": 0.05
    }

    client.update_configuration(config)