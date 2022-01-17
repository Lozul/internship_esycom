#! /usr/bin/env python
import threading

import rospy
import numpy as np

from robot_main.msg import Routine
from std_msgs.msg import Bool, UInt8

r = Routine(1, 0.2, 5.0, 9.0)
pending = False

def sub_progress(msg):
    global pending

    if pending and msg.data == r.nb_steps:
        print '\rRoutine is finished'

        with open("/home/husarion/robot_reports/report.csv", mode='r') as f:
            data = np.genfromtxt(f, delimiter=',', names=True)
            print_data(data)

        pending = False


def print_data(data):
    for name in data.dtype.names:
        print name.rjust(10),

    print ""

    for step in data:
        for d in step:
            print str(d).rjust(10),
        print ""

    print "---"


def main():
    global pending

    rospy.init_node("server", anonymous=True)

    set_routine = rospy.Publisher('set_routine', Routine, queue_size=1)
    start_routine = rospy.Publisher('start_routine', Bool, queue_size=1)
    rospy.Subscriber('routine_progress', UInt8, sub_progress)

    while not rospy.is_shutdown():
        user = raw_input(">>> ")
        if user == "send":
            if not pending:
                set_routine.publish(r)
                start_routine.publish(True)
                pending = True
            else:
                print("Last routine is not finished")
        elif user == "quit":
            break

if __name__ == "__main__":
    main()

