#!/usr/bin/env python

from math import pi

import rospy
from robot_driver.srv import *


def get_correction():
    rospy.wait_for_service('get_correction')
    try:
        get_correction = rospy.ServiceProxy('get_correction', GetCorrection)
        resp = get_correction()
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Requesting correction...")

    resp = get_correction()

    if resp:
        print("[%s]: %s rad (%s deg)"%(resp.success, resp.angle, resp.angle * 180 / pi))
    else:
        print("error")
