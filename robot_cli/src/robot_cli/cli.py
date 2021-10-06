from robot_cli.colors import *
import rospy
import dynamic_reconfigure.client

VERSION = "0.1"

commands = dict()

client = None
config = None

user_input = ""

def get_configuration():
    global config

    config = client.get_configuration()

def register(func):
    """Register a function as a command
    """

    commands[func.__name__.replace("cmd_", "")] = func
    return func

@register
def cmd_show():
    """Show current configuration"""

    get_configuration()

    for key, value in config.items():
        if key == "groups":
            continue

        print("\t{0}: {1} <{2}>".format(key, value, type(value).__name__))

@register
def cmd_help():
    """Help function"""

    print("CLI Tool, {0}\n".format(VERSION))
    print("Let you set the dynamic configuration of the Robot.\n")

    print("List of commands:")
    for name, func in commands.items():
        # print(f"\t{name:<10} {func.__doc__}")
        print("\t{0:<10} {1}".format(name, func.__doc__))

@register
def cmd_quit():
    """Quit the CLI"""

    exit(0)

def main():
    global client

    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("robot_main_node", timeout=30)

    while True:
        # user_input = input(f"{good}> {endc}")
        try:
            user_input = raw_input("{0}> {1}".format(good, endc))
        except EOFError:
            cmd_quit()

        if user_input == 0:
            continue

        cmd = user_input.split(" ")[0]

        if cmd in commands.keys():
            commands[cmd]()
        else:
            print("Invalid command")