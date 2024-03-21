# homebreaker-tutorial
Basic ROS tutorials

# Installation
+
create a ROS workspace

```bash
mkdir -p tutorial_ws/src
```

Clone this repository into the `src` folder.

```bash
cd tutorial_ws/src
git clone https://github.com/uchile-robotics/homebreaker-tutorial.git
```

Now, add the kobuki repository to the workspace on the src folder

```bash
cd tutorial_ws/src
git clone https://github.com/yujinrobot/kobuki
git clone https://github.com/yujinrobot/kobuki_desktop.git
rosinstall . kobuki/kobuki.rosinstall
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

### Connect to the robot

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