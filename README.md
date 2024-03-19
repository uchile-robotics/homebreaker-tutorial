# homebreaker-tutorial
Basic ROS tutorials

# Installation

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
git clone https://github.com/yujinrobot/kobuki.git
rosinstall . kobuki/kobuki.rosinstall
```

Compile.

```bash
cd tutorial_ws
catkin_make
```

### Connect to the robot

```bash
ls -n /dev | grep kobuki
sudo adduser <your-username> dialout
rosrun kobuki_ftdi create_udev_rules
roscd kobuki_ftdi
make udev
```