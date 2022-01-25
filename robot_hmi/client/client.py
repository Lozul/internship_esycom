#! /usr/bin/env python3
from math import floor
import threading
import pickle
import socket
import time
import re

from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog


PORT = 9999


class App(ttk.Frame):
    def __init__(self, root, ipVar, sock):
        ttk.Frame.__init__(self, root)

        self.ipVar = ipVar
        self.socket = sock

        self.lock = threading.Lock()

        self.nb_steps = IntVar(value=10)
        self.step_distance = DoubleVar(value=0.2)

        self.freq_start = DoubleVar(value=6.0)
        self.freq_end = DoubleVar(value=18.0)
        self.power_level = IntVar(value=5)
        self.time = IntVar(value=500)
        self.sweep = BooleanVar(value=False)

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
        mode_no_b = ttk.Radiobutton(mode_frame, text='No sweep',
                variable=self.sweep, value=False)
        mode_sweep_b = ttk.Radiobutton(mode_frame, text='Sweep',
                variable=self.sweep, value=True)

        # Button for start
        start_b = ttk.Button(self, text='Start', command=self.send_routine)

        # Indeterminate progressbar
        self.progress = ttk.Progressbar(self, orient=HORIZONTAL,
                length=100, mode='indeterminate')

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

        mode_no_b.grid(column=0, row=0, sticky=W)
        mode_sweep_b.grid(column=0, row=1, sticky=W)

        start_b.grid(column=0, row=2)

        self.progress.grid(column=0, row=3)

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)

    def send_routine(self):
        # TODO: fix power_level
        if not self.lock.acquire(blocking=False):
            messagebox.showinfo(message="A routine is in progress")
            return

        self.progress.start(5)

        routine = {
            "nb_steps": self.nb_steps.get(),
            "step_distance": self.step_distance.get(),
            "freq_start": int(floor(self.freq_start.get() * 1e8)),
            "freq_end": int(floor(self.freq_end.get() * 1e8)),
            "power_level": int(floor(self.power_level.get() * 4)),
            "time": self.time.get(),
            "sweep": self.sweep.get()
        }

        data = pickle.dumps(routine, protocol=0)

        self.socket.sendall(data)

        threading.Thread(None, self.parse_responses).start()

    def parse_responses(self):
        while True:
            msg = self.socket.recv(2048)
            msg = msg.decode("utf-8")

            if msg == "STEP":
                print("Robot finished a step")
            else:
                print("Received report")
                break

        filename = filedialog.asksaveasfilename(defaultextension=".csv",
                filetypes=[("CSV", "*.csv"), ("All files", "*")])

        if not filename:
            filename = "last_unsaved_report.csv"

        with open(filename, mode="w") as f:
            f.write(msg)

        self.lock.release()
        self.progress.stop()


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

    App(root, ipVar, sock)
    AskIP(root, ipVar, sock)
    
    root.mainloop()


if __name__ == "__main__":
    main()
