# homebreaker-tutorial
This repository contains basic ROS tutorials using the kobuki robot base. You will learn aboout the structure of ROS, the Gazebo simulator, sensors such as LiDAR and high level tools like "robot skills" and state machines.


# Installation
First, you must install ROS. This repository was intended for ROS Melodic on Ubuntu 18.04. However, you may be able to make it work on ROS Noetic with Ubuntu 20.04 with some tweaks.

To install ROS, [follow this link](https://wiki.ros.org/Installation/Ubuntu) (make sure to choose the correct ROS distro).


After the installation, create a ROS workspace

```bash
mkdir -p tutorial_ws/src
```

Clone this repository into the `src` folder and install it's dependencies.

```bash
cd tutorial_ws/src
git clone https://github.com/uchile-robotics/homebreaker-tutorial.git
rosdep install --from-paths src --ignore-src -r -y
```

Now, add the kobuki repository to the workspace on the src folder

```bash
cd tutorial_ws/src
git clone https://github.com/yujinrobot/kobuki_desktop.git
rosinstall . kobuki_desktop/kobuki_simulator.rosinstall
```

You may need to manually install the `ecl` library.

```bash
sudo apt install ros-melodic-ecl
```

Compile.

```bash
cd tutorial_ws
catkin_make
```

We recommend you add the following lines to your .bashrc or .zshrc file (if you haven't already), to prevent you from manually sourcing the files every time you open a terminal.

```bash
source /opt/ros/melodic/setup.bash
source <path/to/workspace>/devel/setup.bash
```

# Getting started

## Simulation

To start the simulation, run the following command on a terminal

```bash
roslaunch gazebo_simulation gazebo.launch
```

This should start Gazebo on a simulated version of the robot inside the Lab's stage, with all it's sensors, like LiDAR, Odometry and webcam.

To learn more about it, follow the [tutorials](tutorials/).

### Connect to the robot
To use the robot via WiFi, you'll need to follow 2 steps.
First, make sure that you are using the same network as the Raspberry Pi 4 onboard (It is probably `UChilehomebreakers`) and connect via ssh to it. The password is 'kobuki'.

```bash
ssh kobuki@kobuki.local
```

Then, you need to configure your ROS network to be able to visualize topics over the internet. For this, add the following lines to your `bashrc` file if you use bash or `zshrc` if you use zsh (don't forget to source the file after updating it).

```bash
export ROS_MASTER_URI= http://kobuki.local:11311
export ROS_HOSTNAME= <your-hostname>.local
export ROS_IP= <your-ip>
```

You can obtain your IP and hostname with the following:

```bash
hostname # for the hostname
hostname -I # for the IP
```


After that you can start the robot using the bringup file, from a terminal **in the robot**. This will start up the base, LiDAR, camera, and localization.


```bash
roslaunch kobuki_bringup kobuki.launch
```

You can visualize the robot using RViz from **your computer** with:

```bash
roslaunch kobuki_bringup rviz.launch
```


### Direct connection to the base
> [!NOTE]
> If you are using the raspberry pi, ignore this section.

Connection to the kobuki base is done via USB. To make it work, you can configure a udev rule to make the base always recognizable to your computer in the port `/dev/kobuki`. 

```bash
ls -n /dev | grep kobuki
sudo adduser <your-username> dialout
rosrun kobuki_ftdi create_udev_rules
roscd kobuki_ftdi
make udev
```

If you don't want to configure an udev rule, you must manually search on which port your kobuki is at. To do so, run the following

```bash
ls -l /dev/serial/by-id
```

You should get an output similar to the one below, where `../../ttyUSB0` means that your kobuki base is connected to the port `/dev/ttyUSB0`. 

```bash
$ ~/ total 0
lrwxrwxrwx ... usb-Yujin_Robot_iClebo_Kobuki_kobuki_A907BTNL-if00-port0 -> ../../ttyUSB0
```

Afterwards, you should modify the `device_port` parameter in the [base.yaml](../kobuki/kobuki_node/param/base.yaml) file.


```yaml
device_port: /your/dev/port
...
```