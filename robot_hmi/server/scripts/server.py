#! /usr/bin/env python
import socket
import pickle

import rospy

from robot_main.msg import Routine
from std_msgs.msg import Bool, UInt8


def pub_routine(routine):
    set_routine = rospy.Publisher('set_routine', Routine, queue_size=1)
    start_routine = rospy.Publisher('start_routine', Bool, queue_size=1)

    while set_routine.get_num_connections() < 1 or start_routine.get_num_connections() < 1:
        continue

    print(set_routine.get_num_connections(), start_routine.get_num_connections())

    set_routine.publish(routine)
    start_routine.publish(True)

def main():
    HOST, PORT = "0.0.0.0", 9999

    rospy.init_node("server", anonymous=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)

    print 'Server is on'

    conn, addr = server.accept()
    print 'Connected by', addr

    while 1:
        data = conn.recv(1024)
        if not data: break

        r = pickle.loads(data)
        routine = Routine(r["nb_steps"], r["step_distance"], r["frequency"], r["power_level"])

        print 'Received routine:'
        print routine

        pub_routine(routine)

        print 'Waiting for routine to end...'
        while rospy.wait_for_message('routine_progress', UInt8).data != routine.nb_steps:
            continue

        report = ""
        with open("/home/husarion/robot_reports/report.csv", mode='r') as f:
            report = f.read()

        print 'Sending report:'
        print report

        conn.sendall(report)
        print 'Report sended!'

if __name__ == "__main__":
    main()

