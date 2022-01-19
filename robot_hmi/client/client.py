#! /usr/bin/env python3
import pickle
import socket
import re

from tkinter import *
from tkinter import ttk
from tkinter import messagebox


PORT = 9999


class App(ttk.Frame):
    def __init__(self, root, ipVar, sock):
        ttk.Frame.__init__(self, root)

        self.ipVar = ipVar
        self.socket = sock

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
        start_b = ttk.Button(self, text='Start', command=self.send_routine)

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

    def send_routine(self):
        routine = {
            "nb_steps": self.nb_steps.get(),
            "step_distance": self.step_distance.get(),
            "frequency": self.freq_start.get(),
            "power_level": self.power_level.get()
        }

        data = pickle.dumps(routine, protocol=0)

        self.socket.sendall(data)


class AskIP(Toplevel):
    def __init__(self, parent, ipVar, sock):
        Toplevel.__init__(self, parent)
        self.title("Robot IP")

        self.ipVar = ipVar
        self.socket = sock

        self.create_widgets()

        self.protocol("WM_DELETE_WINDOW", self.quit)
        self.transient(parent)
        self.grab_set()
        self.wait_window()

    def create_widgets(self):
        ttk.Label(self, text="Enter Robot IP:").grid()

        vcmd = (self.register(self.valid_entry), "%S")
        ttk.Entry(self, textvariable=self.ipVar, validate="key", validatecommand=vcmd).grid()

        ttk.Button(self, text='OK', command=self.valid_ip).grid()

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)

    def valid_entry(self, S):
        return re.match("[.0-9]", S) is not None

    def valid_ip(self):
        ip = self.ipVar.get()
        v = re.fullmatch("(\A25[0-5]|\A2[0-4][0-9]|\A[01]?[0-9][0-9]?)(\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)){3}", ip)

        if v is not None and self.connect_to_ip():
            self.dismiss()
        else:
            messagebox.showerror(message="Please enter a valid IP address", title="Invalid IP")

    def connect_to_ip(self):
        ip = self.ipVar.get()
        try:
            self.socket.connect((ip, PORT))
        except (TimeoutError, ConnectionRefusedError):
            print(f"Can't connect to {ip} on port {PORT}")
            return False

        return True


    def dismiss(self):
        self.grab_release()
        self.destroy()

    def quit(self):
        self.dismiss()
        quit()


def main():
    root = Tk()
    root.title("Robot GUI")

    ipVar = StringVar(value="0.0.0.0")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)

    App(root, ipVar, sock)
    AskIP(root, ipVar, sock)
    
    root.mainloop()


if __name__ == "__main__":
    main()
