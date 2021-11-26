from Tkinter import *
import ttk

import rospy
from robot_main.msg import Routine
from std_msgs.msg import Bool

set_routine = None
start_routine = None

class App(ttk.Frame):
    def __init__(self, master=None):
        ttk.Frame.__init__(self, master)

        self.nb_steps = IntVar()
        self.nb_steps.set(1)

        self.step_distance = DoubleVar()
        self.step_distance.set(1.0)

        self.create_widgets()

        self.grid(column=0, row=0, padx=12, pady=12)

    def create_widgets(self):
        nb_steps_label = ttk.Label(self, text='Nb steps')
        step_dist_label = ttk.Label(self, text='Distance (m)')

        self.nb_steps_entry = ttk.Entry(self, textvariable=self.nb_steps)
        self.step_distance_entry = ttk.Entry(self, textvariable=self.step_distance)

        self.progress = ttk.Progressbar(self, mode='determinate')

        start_button = ttk.Button(self, text='Start', command=self.start_routine)
        stop_button = ttk.Button(self, text='Stop', command=self.stop_routine)

        nb_steps_label.grid(column=0, row=0, sticky=W)
        step_dist_label.grid(column=1, row=0, sticky=W)
        self.nb_steps_entry.grid(column=0, row=1)
        self.step_distance_entry.grid(column=1, row=1)
        start_button.grid(column=2, row=1)
        stop_button.grid(column=3, row=1)
        self.progress.grid(column=0, row=2, columnspan=4, sticky=W+E)

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)

    def start_routine(self):
        nb_steps = int(self.nb_steps_entry.get())
        step_distance = self.step_distance.get()

        print("Routine: {0} steps of {1}m".format(nb_steps, step_distance))
        set_routine.publish(Routine(nb_steps, step_distance))
        start_routine.publish(True)

    def stop_routine(self):
        start_routine.publish(False)

def main():
    global set_routine, start_routine

    # ROS
    rospy.init_node("robot_gui")

    set_routine = rospy.Publisher('set_routine', Routine, queue_size=1)
    start_routine = rospy.Publisher('start_routine', Bool, queue_size=1)

    # Interface
    root = Tk()
    root.title("Robot GUI")

    App(root)

    root.mainloop()

if __name__ == "__main__":
    main()
