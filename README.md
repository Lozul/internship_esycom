# ROS packages for the ESYCOM laboratory

> A set of packages developped for the ESYCOM laboratory to use the ROSBot 2.0 pro of [Husarion](https://husarion.com/)

[TOC]

## Description

The project is composed of 3 packages:
- `robot_driver`: set of methods for basic movements (linear and angular) plus a method to correct the orientation using a laser scan and a target
- `robot_hmi`: pool of tools to control the robot:
    - `robot_gui`: simple tool to set routine for the robot to execute, could be used when connected over SSH with the robot if your system can do X11-forwarding
    - `server` & `client`: the first is launched aside the main program and the second can connect to it to set routine and setup the PNA.
- `robot_main`: central package that executes the other when needed

The main objective is to precisely control the robot to perform measurements semi-automatically to study wireless systems operating close to the ground.

Documentation can be found here:
- RobotDriver for driving methods and Routine for settings
- \subpage GetCorrectionExplained for correction calculations

## Instalation

_client instalation_

## Usage

> This section should only be of interest to users for whom these packages were built.

First, take a quick look at the robot manual [here](https://husarion.com/manuals/rosbot/), especially the part about charging the robot and the [Charging Manual](https://files.husarion.com/docs2/Charging%20manual%20for%20ROSbot.pdf).

**Do not unplug the power supply if the robot is turned on.** It will make the system crash, first turn off the robot, unplug it, then turn it on again.

### 1. Connection with the robot

You will need:
- a computer who can use SSH [1]
- an smartphone running Android to create a hotspot the PNA can connect to
- an RJ45 (ethernet) cable
- a wifi dongle for the PNA

[1] : Instructions for Windows [here](https://docs.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse#install-openssh-using-windows-settings), install the client. The OpenSSH client should be already installed on most Linux systems, if not, search it with your package manager

#### 1.1 Access the robot

1. Wire your computer and the robot, you will need the rj45 to usb adaptator, it should be in one of the two accessory boxes of the robot
2. Using your terminal (Powershell or CMD on Windows), connect through SSH to the robot (it have to be turned on, look for two red leds on the back)

```
$ ssh husarion@192.168.0.1
```

Password is `husarion`.

#### 1.2 Connect to wifi

**University wifi, eduroam, can not be used here.**

1. Turn on hotspot on the Android (https://support.google.com/android/answer/9059108?hl=fr)
2. Connect the robot to the shared network with the command `nmtui`.
3. Connect your computer to the hotspot.
4. Plug the Wifi dongle in the PNA and connect it to the hotspot.

#### 1.3 Get Robot IP address

In your terminal connect through SSH to the robot, type `ifconfig` to find the robot IP adress (should be under "wlan..."). It sould look like this : `10.42.0.23`.

You can now close the SSH connection and unplug the ethernet cable.

#### 1.4 Get PNA IP address

1. Open a terminal on the PNA (Windows + r then type cmd and enter)
2. Type `ipconfig` (note it is **ip** not **if**)
3. Note the ip address under "Wi-Fi"

#### 1.4 Wireless connection to the robot

In your PC terminal, type:

```
$ ssh husarion@ROBOT_IP
```

Replace `ROBOT_IP` with the one found with the `ifconfig` command.

You now have a wireless connection with the robot.

### 2. Starting the robot software

To start the robot software, use this command in the robot terminal:

```
$ roslaunch robot_main drive.launch
```

### 3. How to use the client

![](ihm.png)

1. Enter the IP addresses of the robot and of the PNA, click OK.
2. The right window let you setup the PNA, enter the parameters you want and click on the `Apply` button.
3. Check on the PNA that everything is alright
4. The window in the center let you setup a routine and send it to the robot, use the entries and click on the `Start` button when ready.
5. Wait for the robot to finish is routine, the loading bar will stop moving when the client received the data.
6. You can find the data in the folder `routine_reports` (located at the same place that the `client.py`). Search the last created folder inside.

#### 3.1 In case of problem

**One of the software (client or robot) crashed:**
- Close the client.
- Stop the robot software (Ctrl+C in the terminal where you started it).
- Turn off the robot, wait a few seconds, then turn it on again.
- Go back to 1.4, the different devices should reconnect to the hotspot.

### 4. How to read the data

Each routine report will be composed of CSV files.

The first one is `robot.csv`, for each steps you will have the final angle after correction and the distance traveled according to the laser and the encoders.

The other, `pna_X.csv`, contains the measurements made at each steps with the PNA.

## License

Software to use the signal generator (LMS-183DX) was provided by Vaunix, see [LMShid.c](robot_main/src/LMShid.c) and [LMShid.h](robot_main/include/LMShid.h) for copyright.

## Credits

Intern developper: Louis Gasnault

Supervisors: Benoit Poussot and Shermila Mostarshedi
