# Name - ROS packages for the ESYCOM laboratory

> A set of packages developped for the ESYCOM laboratory to use the ROSBot 2.0 pro of [Husarion](https://husarion.com/)

## Description

The project is composed of 3 packages:
- `robot_driver`: set of methods for basic movements (linear and angular) plus a method to correct the orientation using a laser scan and a target
- `robot_gui`: simple tool to set routine for the robot to execute, could be used when connected over SSH with the robot if your system can do X11-forwarding
- `robot_main`: central package that executes the other when needed

The main objective is to precisely control the robot to perform measurements semi-automatically to study wireless systems operating close to the ground.

Details about how each package works will be available in their respective documentation (soon™).

## Usage

> This section should only be of interest to users for whom these packages were built.

First, take a quick look at the robot manual [here](https://husarion.com/manuals/rosbot/), especially the part about charging the robot and the [Charging Manual](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf).

**Do not unplug the power supply if the robot is on.** (It will make the system crash).

**This part should be updated when the new GUI tool will be available.**

### 1. Connection with the robot

You will need:
- a computer who can use SSH (instructions for Windows [here](https://docs.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse#install-openssh-using-windows-settings), install the client, the OpenSSH client should be already installed on most Linux systems, if not, search it with your package manager)
- a wifi network on which the robot can connect **or** the ability to share the wifi of your computer
- an RJ45 (ethernet) cable **or** a screen, a keyboard and a mouse

#### 1.1 Access the robot

**Using Ethernet cable:**

1. Wire your computer and the robot, you will need the rj45 to usb adaptator, it should be in one of the two accessory boxes of the robot
2. Using your terminal, connect through SSH to the robot (it have to be turned on, loof for two red leds on the back)

```
$ ssh husarion@192.168.0.1
```

Password is `husarion`, when logged in, go to 1.2.

**Using screen, keyboard and mouse:**

1. Connect all the devices to the robot
2. Turn the robot on, the system should boot up and display on the screen

#### 1.2 Connect to wifi

_University wifi, eduroam, can not be used here._

If you have a wifi the robot can connect to, use `nmtui` or the graphical tool (top right of the desktop) to connect the robot.

Else follow this instructions:

1. Share the wifi of your computer
    - on [Windows](https://support.microsoft.com/en-us/windows/use-your-windows-pc-as-a-mobile-hotspot-c89b0fad-72d5-41e8-f7ea-406ad9036b85)
    - on linux it depends of your system
2. Connect the robot to the shared network with `nmtui` or the graphical tool.

#### 1.3 Wireless connection to the robot

In the terminal, type `ifconfig` to find the robot IP address.

You can now open a terminal on your computer and connect through SSH to the robot using the IP address:

```
$ ssh husarion@ROBOT_IP
```

If your system can do X11-forwarding, you can add the `-X` parameter.

Replace `ROBOT_IP` with the one found with `ifconfig` command.

You now have a wireless connection with the robot.

### 2. Starting the software

**Warning, in his current state, the soft need to open a graphical interface. You have to be able to do X11-forwarding.**

To start the software, use this command:

```
$ roslaunch robot_main drive.launch
```

### 3. Using the software

**TODO**

## Roadmap

- [ ] documentation of the project
- [ ] communication with the receptor
- [ ] graphical tool to set robot's routine from another machine

## License

Software to use the signal generator (LMS-183DX) was provided by Vaunix, see [LMShid.c](robot_main/src/LMShid.c) and [LMShid.h](robot_main/include/LMShid.h) for copyright.

## Credits

Intern developper: Louis Gasnault

Supervisors: Benoit Poussot and Shermila Mostarshedi
