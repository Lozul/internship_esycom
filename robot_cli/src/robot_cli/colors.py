# Colors
endc = "\033[0m"
bold = "\033[1m"

error = "\033[91m"
good = "\033[92m"
warn = "\033[93m"

# Colored print
def perror(msg):
    print("{0}{1}{2}".format(error, msg, endc))

def pgood(msg):
    print("{0}{1}{2}".format(good, msg, endc))

def pwarn(msg):
    print("{0}{1}{2}".format(warn, msg, endc))
