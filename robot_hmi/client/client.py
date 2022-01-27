#! /usr/bin/env python3
from math import floor
from pathlib import Path
import threading
import pickle
import socket
import time
import re

from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog

import pyvisa


PORT = 9999
RM = pyvisa.ResourceManager()
VALID_IP = "(\A25[0-5]|\A2[0-4][0-9]|\A[01]?[0-9][0-9]?)(\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)){3}"


class App(ttk.Frame):
    def __init__(self, root):
        ttk.Frame.__init__(self, root)

        self.robot_ip = StringVar(value="0.0.0.0")
        self.pna_ip = StringVar(value="0.0.0.0")

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pna = None

        self.lock = threading.Lock()

        self.nb_steps = IntVar(value=10)
        self.step_distance = DoubleVar(value=0.2)

        self.freq_start = DoubleVar(value=6.0)
        self.freq_end = DoubleVar(value=18.0)
        self.power_level = IntVar(value=5)
        self.time = IntVar(value=500)
        self.sweep = BooleanVar(value=False)

        self.pna_freq_cent = IntVar(value=int(6e9))
        self.pna_freq_span = IntVar(value=int(10e6))
        self.pna_bandwidth = IntVar(value=int(3.5e4))
        self.pna_nb_points = IntVar(value=201)

        self.create_widgets()

        self.grid(column=0, row=0, padx=12, pady=12)

        SetupPNA(self)
        AskIP(self)

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

        self.log_dir = f"routine_reports/{floor(time.time())}"
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)

        threading.Thread(None, self.parse_responses).start()

    def setup_pna(self):
        # Reset
        self.pna.write("*rst; status:preset; *cls")

        # Modifying default measure for our needs
        name = self.pna.query("calc1:par:cat?").replace('"', '').split(",")[0]
        self.pna.write(f"calc1:par:sel {name}")
        self.pna.write("calc1:par:mod:ext 'B,2'")

        # Set center frequence and span
        self.pna.write(f"sens1:freq:cent {6e9}")
        self.pna.write(f"sens1:freq:span {10e6}")

        # Set IF Bandwidth
        self.pna.write(f"sens1:bwid {3.5e4}")

        # Set nb points
        self.pna.write(f"sens1:swe:poin {201}")

    def parse_responses(self):
        done = False
        step = 1

        while not done:
            msg = self.socket.recv(2048)
            msg = msg.decode("utf-8")

            if msg == "STEP":
                print("Robot finished a step")
                filename = f"{self.log_dir}/pna_{step}.csv"
                step += 1

                msg = self.pna.query("calc1:data? fdata").replace(',', '\n')
            else:
                print("Received report")
                filename = f"{self.log_dir}/robot.csv"
                done = True

            with open(filename, mode="w") as f:
                f.write(msg)

        self.lock.release()
        self.progress.stop()


class AskIP(Toplevel):
    def __init__(self, parent):
        Toplevel.__init__(self, parent)
        self.title("Robot IP")

        self.parent = parent

        self.create_widgets()

        self.protocol("WM_DELETE_WINDOW", self.quit)
        self.transient(parent)
        self.grab_set()
        self.wait_window()

    def create_widgets(self):
        vcmd = (self.register(self.valid_entry), "%S")

        ttk.Label(self, text="Enter Robot IP:").grid()
        ttk.Entry(self, textvariable=self.parent.robot_ip, validate="key", validatecommand=vcmd).grid()

        ttk.Label(self, text="Enter PNA IP:").grid()
        ttk.Entry(self, textvariable=self.parent.pna_ip, validate="key", validatecommand=vcmd).grid()

        ttk.Button(self, text='OK', command=self.valid_ip).grid()

        for child in self.winfo_children():
            child.grid(padx=5, pady=5)

    def valid_entry(self, S):
        return re.match("[.0-9]", S) is not None

    def valid_ip(self):
        robot_ip = self.parent.robot_ip.get()
        pna_ip = self.parent.pna_ip.get()

        if not re.fullmatch(VALID_IP, robot_ip):
            messagebox.showerror(message="Please enter a valid IP address for robot", title="Invalid Robot IP")
        elif not re.fullmatch(VALID_IP, robot_ip):
            messagebox.showerror(message="Please enter a valid IP address for PNA", title="Invalid PNA IP")
        elif not self.connect_to_server():
            messagebox.showerror(message="Can not connect to server with this IP", title="Invalid Robot IP")
        elif not self.connect_to_pna():
            messagebox.showerror(message="Can not connect to PNA with this IP", title="Invalid PNA IP")
        else:
            self.dismiss()        
            

    def connect_to_server(self):
        ip = self.parent.robot_ip.get()
        try:
            self.parent.socket.connect((ip, PORT))
        except:
            print(f"Can't connect to {ip} on port {PORT}")
            return False

        return True

    def connect_to_pna(self):
        ip = self.parent.pna_ip.get()
        self.parent.pna = RM.open_resource(f"TCPIP0::{ip}::5025::SOCKET")

        try:
            pna.write_termination, pna.read_termination = "\n", "\n"
            response = pna.query("*idn?")
            print(f"Connect to PNA: {response}")
        except:
            return False

        return True

    def dismiss(self):
        self.grab_release()
        self.destroy()

    def quit(self):
        self.dismiss()
        quit()


class SetupPNA(Toplevel):
    def __init__(self, parent):
        Toplevel.__init__(self, parent)
        self.title("PNA Setup")

        self.parent = parent

        self.create_widgets()

    def create_widgets(self):
        ttk.Label(self, text="Center frequence (Hz)").grid()
        ttk.Entry(self, textvariable=self.parent.pna_freq_cent).grid()

        ttk.Label(self, text="Span frequence (Hz)").grid()
        ttk.Entry(self, textvariable=self.parent.pna_freq_span).grid()

        ttk.Label(self, text="IF Bandwidth (Hz)").grid()
        ttk.Entry(self, textvariable=self.parent.pna_bandwidth).grid()

        ttk.Label(self, text="Nb points").grid()
        ttk.Entry(self, textvariable=self.parent.pna_nb_points).grid()

        ttk.Button(self, text="Apply", command=self.apply_setup).grid()

        for child in self.winfo_children():
            child.grid(padx=25, pady=5)

    def apply_setup(self):
        # Reset
        pna.write("*rst; status:preset; *cls")

        # Modifying default measure for our needs
        name = pna.query("calc1:par:cat?").replace('"', '').split(",")[0]
        pna.write(f"calc1:par:sel {name}")
        pna.write("calc1:par:mod:ext 'B,2'")

        # Set center frequence and span
        pna.write(f"sens1:freq:cent {self.parent.pna_freq_cent.get()}")
        pna.write(f"sens1:freq:span {self.parent.pna_freq_span.get()}")

        # Set IF Bandwidth
        pna.write(f"sens1:bwid {self.parent.pna_bandwidth.get()}")

        # Set nb points
        pna.write(f"sens1:swe:poin {self.parent.pna_nb_points.get()}")


def main():
    # Tkinter setup
    root = Tk()
    root.title("Robot GUI")

    # Build GUI
    App(root)
    
    # Loop
    root.mainloop()


if __name__ == "__main__":
    main()
