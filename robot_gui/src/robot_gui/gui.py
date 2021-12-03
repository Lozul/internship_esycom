from Tkinter import *
import ttk

from ctypes import *
from math import floor
from time import sleep

import rospy
from robot_main.msg import Routine
from std_msgs.msg import Bool, UInt8

from lmshid_bindings import lib
from generator_gui import GeneratorGUI

set_routine = None
start_routine = None

# ttk.Spinbox is missing in Python 3.6
class Spinbox(ttk.Entry):
    def __init__(self, master=None, **kw):
        ttk.Entry.__init__(self, master, "ttk::spinbox", **kw)
    def set(self, value):
        self.tk.call(self._w, "set", value)

class GUI(ttk.Frame):
    def __init__(self, root, source_id):
        ttk.Frame.__init__(self, root)

        self.source_id = source_id
        self.routine = Routine()

        self.nb_steps = IntVar()
        self.nb_steps.set(1)
        self.nb_steps.trace("w", self.nb_steps_changed)

        self.step_distance = DoubleVar()
        self.step_distance.set(1.0)

        self.progress_var = IntVar()
        self.progress_var.set(0)

        self.create_widgets()

        self.grid(column=0, row=0, padx=12, pady=12)

    def create_widgets(self):
        nb_steps_label = ttk.Label(self, text='Nb steps')
        step_dist_label = ttk.Label(self, text='Distance (m)')

        self.nb_steps_entry = ttk.Entry(self, textvariable=self.nb_steps)
        self.step_distance_entry = ttk.Entry(self, textvariable=self.step_distance)

        freq_select_button = ttk.Button(self, text='Freq', command=self.ask_freq)

        self.progress = ttk.Progressbar(self, mode='determinate', variable=self.progress_var, maximum=self.nb_steps.get())

        start_button = ttk.Button(self, text='Start', command=self.start_routine)
        stop_button = ttk.Button(self, text='Stop', command=self.stop_routine)

        nb_steps_label.grid(column=0, row=0, sticky=W)
        step_dist_label.grid(column=1, row=0, sticky=W)
        self.nb_steps_entry.grid(column=0, row=1)
        self.step_distance_entry.grid(column=1, row=1)
        freq_select_button.grid(column=2, row=0, columnspan=2)
        start_button.grid(column=2, row=1)
        stop_button.grid(column=3, row=1)
        self.progress.grid(column=0, row=3, columnspan=4, sticky=W+E)

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)

    def start_routine(self):
        self.routine.nb_steps = int(self.nb_steps_entry.get())
        self.routine.step_distance = self.step_distance.get()

        set_routine.publish(self.routine)
        start_routine.publish(True)

    def stop_routine(self):
        start_routine.publish(False)

    def step_routine(self, msg):
        self.progress_var.set(msg.data)

    def ask_freq(self):
        def dismiss():
            self.routine.frequency = r.frequency
            self.routine.power_level = r.power_level
            dlg.grab_release()
            dlg.destroy()

        r = Routine()

        dlg = Toplevel(self)
        gen_gui = GeneratorGUI(dlg, self.source_id, r)
        
        ttk.Button(dlg, text='Done', command=dismiss).grid(padx=12, pady=12)

        dlg.protocol("WM_DELETE_WINDOW", dismiss)
        dlg.wait_visibility()
        dlg.grab_set()

    def nb_steps_changed(self, *args):
        try:
            self.progress['maximum'] = self.nb_steps.get()
            self.progress_var.set(0)
        except ValueError:
            pass

def main():
    global set_routine, start_routine

    test_mode = False
    devices_array = c_uint * 10

    # ROS
    rospy.init_node("robot_gui")

    set_routine = rospy.Publisher('set_routine', Routine, queue_size=1)
    start_routine = rospy.Publisher('start_routine', Bool, queue_size=1)

    # LMS
    lib.fnLMS_Init()
    lib.fnLMS_SetTestMode(test_mode)

    if test_mode:
        print("Running GUI in test mode")

    num_devices = lib.fnLMS_GetNumDevices()
    
    active_devices = devices_array()
    num_actives = lib.fnLMS_GetDevInfo(active_devices)

    if num_actives == 0:
        print("No active device")
        return

    source_id = active_devices[0]

    status = lib.fnLMS_InitDevice(source_id)
    print("Source: {0}".format(lib.fnLMS_perror(status).decode('UTF-8')))

    print("Please wait.")
    sleep(1)

    # Interface
    root = Tk()
    root.title("Robot GUI")

    gui = GUI(root, source_id)

    rospy.Subscriber('routine_progress', UInt8, gui.step_routine)

    root.mainloop()

if __name__ == "__main__":
    main()
