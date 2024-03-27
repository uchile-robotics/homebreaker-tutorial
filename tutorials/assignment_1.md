# Tutorial: Teleoperation Node with Position Saving in ROS

## Introduction to ROS
ROS (Robot Operating System) is a flexible framework for writing robot software. It provides a wide range of tools, libraries, and conventions to help you build complex robot applications. ROS is not an operating system in the traditional sense; rather, it provides services designed for heterogeneous computer clusters, and it aims to simplify code reuse between robotics projects.

### Structure of ROS
ROS follows a distributed computing architecture, where nodes communicate with each other via topics, services, and actions. Nodes are processes that perform computation, while topics are named buses over which nodes exchange messages. Services allow nodes to send requests and receive responses, and actions provide a way to execute long-running tasks asynchronously.

### Why ROS?
- **Modularity**: ROS is highly modular, allowing you to break down complex systems into smaller, manageable components.
- **Community Support**: With a large and active community, ROS provides access to a wealth of resources, including libraries, packages, and tutorials.
- **Platform Independence**: ROS supports multiple operating systems and hardware platforms, making it versatile and adaptable to different robot configurations.

For more info, you can check the official [ROS website](https://www.ros.org/) and the following videos.

[![What is ROS?](https://i.ytimg.com/vi/8QfI5a7lTKU/hqdefault.jpg?sqp=-oaymwEcCNACELwBSFXyq4qpAw4IARUAAIhCGAFwAcABBg==&rs=AOn4CLARV7EtmwFEk3Eby71gqtfjZh1C3A)](https://www.youtube.com/watch?v=8QfI5a7lTKU&t=283s "What is ROS, When to use it, and Why ? - Robot Operating System Tutorial")

[![](https://i.ytimg.com/an_webp/rIrZ11UJHt0/mqdefault_6s.webp?du=3000&sqp=CKHMjbAG&rs=AOn4CLAaAXPMWV2ufTzwCQ2AXMH1wqT1aw)](https://www.youtube.com/watch?v=rIrZ11UJHt0
 "04 - What is Master, Node and Topic in ROS? Information for Beginners")

[![](https://i.ytimg.com/an_webp/KAASuA3_4eg/mqdefault_6s.webp?du=3000&sqp=CPCFjrAG&rs=AOn4CLBAV3UnLoS-K410DWd96CXwX-qgHQ)](https://www.youtube.com/watch?v=KAASuA3_4eg
 "10 things you need to know about ROS! | Getting Ready to Build Robots with ROS #4")



## Objective
In this tutorial, we'll create a teleoperation node in ROS that allows you to control a robot in simulation using an Xbox joystick. Additionally, we'll implement functionality to save robot positions using specific buttons on the joystick.


>## Main Goals
>- Understand the basics of creating a ROS node.
>- Learn how to subscribe to joystick and pose messages.
>- Implement teleoperation functionality with velocity control.
>- Add functionality to save robot positions to a file.

## Background: Teleoperation Node
We've provided a Python script in the [scripts](../homebreaker_tutorial/scripts/) folder that serves as a template for creating a teleoperation node in ROS. This script subscribes to joystick messages and AMCL (a localization algorithm) pose messages and publishes velocity commands to control the robot's movement. Your task is to complete the code to enable teleoperation and position saving functionality.


## Instructions
1. **Understand the simulation**: Before programming, launch the simulation. If you installed everything correctly, just run in a terminal:

```bash
roslaunch gazebo_simulation.launch
```

You should see a screen pop up with the robot inside a room similar to the Lab's stage. 

You can use the following commands to check different topics and information available.

```bash
rosnode list # shows all the running nodes
rosnode info <node-name> # print info about a node
rostopic list # prints all the available topics
rostopic info <topic-name> # print info about a topic
rostopic echo <topic-name> # Listen to messages on a topic
rostopic pub <topic-name> args # Publish a message to a topic
```

Take your time to see all the topics available, such as those from the Joy node, the LiDAR and mobile base. 

2. **Understanding the Code**: Before proceeding, understand the provided code structure and its purpose. Take note of the subscribers, publisher, and callback methods defined in the `Joy_Template` class.

3. **Completing the Teleoperation Functionality**:
   - In the `joy_callback()` method, implement the teleoperation logic based on the joystick input. Use the joystick axes and buttons to control the linear and angular velocity of the robot. To choose what index of button or axis to use, do a rostopic echo to the joy topic and see how tha values change when pressing buttons.
   - Ensure that pressing the appropriate buttons stops the robot's movement (`STOP Button`) and saves the robot's current pose (`Add pose button`).
   - In the `amcl_callback()` method, choose the right fields of the message to store in the position variables.

4. **Saving Positions**:
   - Implement the logic in the `joy_callback()` method to save the current robot pose when a specific button is pressed (`Save ALL poses to file button`).
   - Use the `np.save()` function to save the list of poses to a NumPy binary file.

5. **Testing**:
   - Re-start the simulation and run in another terminal the provided script after completing the necessary parts with:
   ```bash
    rosrun homebreaker_tutorial joy_template.py
   ```
   - Verify that the teleoperation functionality works as expected by controlling the robot's movement with the joystick.
   - Check that pressing the designated buttons stops the robot and saves its poses correctly.

## Summary
In this tutorial, you learned how to create a teleoperation node in ROS that allows you to control a robot using a joystick. You also implemented functionality to save robot positions to a file. By completing this tutorial, you've gained practical experience in ROS node creation and message handling, laying the groundwork for more complex robot applications in the future.