from ctypes import *

import os

# Load the shared library
lib_name = os.path.dirname(__file__) + "/LMShid.so"
lib = CDLL(lib_name)

# Settings
lib.fnLMS_perror.restype = c_char_p
