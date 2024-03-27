# Tutorial 4: Deployment in real robot

## Introduction
In this tutorial, we'll cover the deployment of the ROS nodes that you already developed in the previous tutorials onto a real robot. This process involves transferring the code to the robot's onboard computer, setting up the necessary dependencies.


>## Main Goals
>- Connect to the real robot
>- Repeat all the tasks using the real robot


## Background
Before proceeding, ensure that you have access to the real robot's onboard computer and that it is set up with ROS and necessary dependencies. Additionally, make sure that the robot's hardware, such as the joystick and odometry sensors, is properly configured and functional.

### Steps Overview
1. Turn on the robot and Raspberry Pi
2. Make sure the base and LiDAR arfe connected to the Raspberry Pi
3. Configure your network as explained in the README to connect to the robot.

For each test, instead of running the simulation, you should run the robot's bringup launch. 

```bash
roslaunch kobuki_bringup kobuki.launch
```

Remember to run this **in a terminal inside the robot's computer**. Also, when running the navigation, do it in the raspberry pi as well.

Only RViz and the programmed python nodes should be run in your computer. If you encouter connection issues, then trtansfer the nodes to the robot (with help from the tutors) and run everything locally. 

## Summary
In this tutorial, you learned how to deploy a ROS node onto a real robot, enabling teleoperation and position saving functionalities. By following the steps outlined above, you can effectively transfer your ROS code from development to deployment, allowing your robot to perform the desired tasks autonomously in the real world.