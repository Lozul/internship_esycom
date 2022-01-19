#! /usr/bin/env python3
#import socket
#import pickle
#
#HOST, PORT = '10.42.0.23', 9999
#routine = {"nb_steps": 1, "step_distance": 0.2, "frequency": 5, "power_level": 9}
#data = pickle.dumps(routine, protocol=0)
#
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect((HOST, PORT))
#s.sendall(data)
#data = s.recv(2048)
#s.close()
#
#print('Received', repr(data))

from tkinter import *
from tkinter import ttk


class GUI(ttk.Frame):
    def __init__(self, root):
        ttk.Frame.__init__(self, root)

        self.nb_steps = IntVar(value=10)
        self.step_distance = DoubleVar(value=0.2)

        self.freq_start = DoubleVar(value=6.0)
        self.freq_end = DoubleVar(value=12.0)
        self.power_level = IntVar(value=9)
        self.time = IntVar(value=200)
        self.repeat = BooleanVar(value=False)

        self.create_widgets()

        self.grid(column=0, row=0, padx=12, pady=12)

    def create_widgets(self):
        # Label frames
        drive_frame = ttk.Labelframe(self, text='Drive')
        source_frame = ttk.Labelframe(self, text='Source')
        mode_frame = ttk.Labelframe(source_frame, text='Mode')

        # Labels
        labels = [
            ("Nb steps", drive_frame),
            ("Step distance (m)", drive_frame),
            ("Freq start (GHz)", source_frame),
            ("Freq end (GHz)", source_frame),
            ("Power level (dBm)", source_frame),
            ("Time (ms)", source_frame)
        ]

        for i in range(len(labels)):
            labels[i] = ttk.Label(labels[i][1], text=labels[i][0])

        # Entries
        entries = [
                (self.nb_steps, drive_frame),
                (self.step_distance, drive_frame),
                (self.freq_start, source_frame),
                (self.freq_end, source_frame),
                (self.power_level, source_frame),
                (self.time, source_frame)
        ]

        for i in range(len(entries)):
            entries[i] = ttk.Entry(entries[i][1], textvariable=entries[i][0])

        # Buttons for mode
        mode_one_b = ttk.Radiobutton(mode_frame, text='One time',
                variable=self.repeat, value=False)
        mode_repeat_b = ttk.Radiobutton(mode_frame, text='Repeat',
                variable=self.repeat, value=True)
        mode_repeat_b.state(['disabled'])

        # Button for start
        start_b = ttk.Button(self, text='Start')

        # Grid
        drive_frame.grid(column=0, row=0,sticky=(W, E))
        source_frame.grid(column=0, row=1)
        mode_frame.grid(column=2, row=0, rowspan=4)

        labels[0].grid(column=0, row=0, sticky=W)
        labels[1].grid(column=1, row=0, sticky=W)

        labels[2].grid(column=0, row=0, sticky=W)
        labels[3].grid(column=1, row=0, sticky=W)
        labels[4].grid(column=0, row=2, sticky=W)
        labels[5].grid(column=1, row=2, sticky=W)

        entries[0].grid(column=0, row=1)
        entries[1].grid(column=1, row=1)

        entries[2].grid(column=0, row=1)
        entries[3].grid(column=1, row=1)
        entries[4].grid(column=0, row=3)
        entries[5].grid(column=1, row=3)

        mode_one_b.grid(column=0, row=0, sticky=W)
        mode_repeat_b.grid(column=0, row=1, sticky=W)

        start_b.grid(column=0, row=2)

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)


def main():
    root = Tk()
    root.title("Robot GUI")

    GUI(root)
    
    root.mainloop()


if __name__ == "__main__":
    main()
