#! /usr/bin/env python
from ctypes import *
from time import sleep
import socket
import pickle

import rospy
from robot_main.msg import Routine
from std_msgs.msg import Bool, UInt8

from lmshid_bindings import lib

def perror(status):
    return lib.fnLMS_perror(status).decode('UTF-8')

def get_generator_settings(source_id):
    res = {}

    res["max_pwr"] = lib.fnLMS_GetMaxPwr(source_id) / 4
    res["min_pwr"] = lib.fnLMS_GetMinPwr(source_id) / 4
    res["max_freq"] = lib.fnLMS_GetMaxFreq(source_id) / 1e8
    res["min_freq"] = lib.fnLMS_GetMinFreq(source_id) / 1e8

    return res

def setup_generator(routine, source_id):
    print '-- Setup of generator --'

    status = lib.fnLMS_SetRFOn(source_id, True)
    print ':: Set RF On: {0}'.format(perror(status))

    status = lib.fnLMS_SetPowerLevel(source_id, routine.power_level)
    print ':: Power level: {0}'.format(perror(status))

    if routine.sweep:
        status = lib.fnLMS_SetStartFrequency(source_id, routine.freq_start)
        print ':: Set freq start: {0}'.format(perror(status))

        status = lib.fnLMS_SetEndFrequency(source_id, routine.freq_end)
        print ':: Set freq end: {0}'.format(perror(status))

        status = lib.fnLMS_SetSweepTime(source_id, routine.time)
        print ':: Set sweep time: {0}'.format(perror(status))

        status = lib.fnLMS_SetSweepMode(source_id, True)
        print ':: Set sweep repeat: {0}'.format(perror(status))

        status = lib.fnLMS_SetSweepType(source_id, True)
        print ':: Set sweep bidirectional: {0}'.format(perror(status))

        status = lib.fnLMS_StartSweep(source_id, True)
        print ':: Start sweep: {0}'.format(perror(status))
    else:
        status = lib.fnLMS_SetFrequency(source_id, routine.freq_start)
        print ':: Set frequency: {0}'.format(perror(status))

    print '-- End of generator setup --'

def pub_routine(routine):
    print '-- Publishing routine --'
    set_routine = rospy.Publisher('set_routine', Routine, queue_size=1)
    start_routine = rospy.Publisher('start_routine', Bool, queue_size=1)

    print ':: Waiting for subscribers on set and start routine...'
    while set_routine.get_num_connections() < 1 or start_routine.get_num_connections() < 1:
        continue

    print ':: Sending set and start messages'
    set_routine.publish(routine)
    start_routine.publish(True)

def main():
    # Setup python server
    HOST, PORT = "0.0.0.0", 9999

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    # Init ROS node
    use_routine = False
    rospy.init_node("server", anonymous=True)

    # Init signal generator
    source_id = -1
    devices_array = c_uint * 10
    lib.fnLMS_Init()
    lib.fnLMS_SetTestMode(False)

    num_devices = lib.fnLMS_GetNumDevices()
    
    active_devices = devices_array()
    num_actives = lib.fnLMS_GetDevInfo(active_devices)

    use_generator = num_actives == 1

    if use_generator:
        print ':: Source detected'
        source_id = active_devices[0]

        status = lib.fnLMS_InitDevice(source_id)
        print ":: Source status: {0}".format(perror(status))

        print ":: Please wait."
        sleep(1)

        res = get_generator_settings(source_id)
        print ":: Source settings"
        print res
    else:
        print ':: No generator detected'

    print '-- Server is on --'

    # Wait for connection
    conn, addr = server.accept()
    print '-> Connected by', addr

    while 1:
        data = conn.recv(1024)
        if not data: break

        # Convert data to robot_main::Routine
        r = pickle.loads(data)
        routine = Routine(r["nb_steps"], r["step_distance"], r["freq_start"], r["freq_end"], r["power_level"], r["time"], r["sweep"])

        print '-> Received routine:'
        print routine

        if use_generator:
            setup_generator(routine, source_id)

        if use_routine:
            pub_routine(routine)

            print ':: Waiting for routine to end...'
            while rospy.wait_for_message('routine_progress', UInt8).data != routine.nb_steps and routine.nb_steps != 0:
                continue

        report = ""
        with open("/home/husarion/robot_reports/report.csv", mode='r') as f:
            report = f.read()

        print '<- Sending report:'

        conn.sendall(report)
        print ':: Report sended!'

if __name__ == "__main__":
    main()

