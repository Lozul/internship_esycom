from subprocess import Popen, PIPE
from time import sleep

import numpy as np

line = "-" * 40
print("\033[1;92m .Starting roslaunch subprocess\n{0}\033[0m".format(line))

p = Popen('roslaunch robot_main drive.launch', shell=True)

p.wait()

print("\033[1;92m{0}\n .Subprocess finished\033[0m".format(line))
