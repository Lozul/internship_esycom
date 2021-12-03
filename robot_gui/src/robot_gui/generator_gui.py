from ctypes import *
from Tkinter import *
from math import floor

import ttk

from robot_main.msg import Routine

from lmshid_bindings import lib

def to_ghz(freq_10hz):
    return freq_10hz / 1.0e8

class GeneratorGUI(ttk.Frame):
    def __init__(self, root, source_id, routine):
        ttk.Frame.__init__(self, root)

        # Routine object to return new generator settings
        self.routine = routine

        # Generator current settings
        # --- Frequency
        self.freq_var = DoubleVar()
        self.freq_var.set(to_ghz(lib.fnLMS_GetFrequency(source_id)))
        self.freq_var.trace("w", self.freq_trace)

        self.freq_min = to_ghz(lib.fnLMS_GetMinFreq(source_id))
        self.freq_max = to_ghz(lib.fnLMS_GetMaxFreq(source_id))

        # --- Power Level
        self.pow_var = IntVar()
        self.pow_var.set(lib.fnLMS_GetPowerLevel(source_id) * 0.25)
        self.pow_var.trace("w", self.pow_trace)

        self.pow_min = lib.fnLMS_GetMinPwr(source_id) * 0.25
        self.pow_max = lib.fnLMS_GetMaxPwr(source_id) * 0.25

        # Widgets
        self.create_widgets()

        self.grid(column=0, row=0, padx=12, pady=12)

    def create_widgets(self):
        # Frames
        freq_frame = ttk.LabelFrame(self, text='Freq (GHz)', borderwidth=0) 
        pow_frame = ttk.LabelFrame(self, text='Power (dBm)', borderwidth=0)

        # Frequency
        freq_spin = Spinbox(freq_frame, from_=self.freq_min, to=self.freq_max, increment=0.1, textvariable=self.freq_var, format="%.3f") 

        # Power Level
        pow_spin = Spinbox(pow_frame, from_=self.pow_min, to=self.pow_max, increment=1, textvariable=self.pow_var) 

        # Grid
        freq_frame.grid(column=0, row=0, sticky=W)
        pow_frame.grid(column=1, row=0, sticky=E)

        freq_spin.grid(column=0, row=0)
        pow_spin.grid(column=0, row=0)

    def freq_trace(self, *args):
        try:
            int(self.freq_var.get())
        except ValueError:
            return

        self.routine.frequency = int(floor(self.freq_var.get() * 1e8))

    def pow_trace(self, *args):
        try:
            int(self.pow_var.get())
        except ValueError:
            return

        self.routine.power_level = int(floor(self.pow_var.get() * 4))
