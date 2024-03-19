<!-- # Tutorial 2: SMACH middleware

The objective of this tutorial is to create a module that make an abstraction from the ROS navigation stack, so the use of it is simplified within a state machine.

> **Tutorial goals:**
> - Learn about the navigation stack
> - Use the `MoveBaseClient` service
> - Create an Object Oriented "skill"


`MoveBaseClient` 


```python
import rospy
from ... import MoveBaseClient
...


class BaseSkill(object):
    def __init__(self, **kwargs):
        pass
``` -->

# Tutorial 2: Creating a Middleware between the Navigation Stack and SMACH using MoveBaseClient

## Objective
In this tutorial, we aim to create a middleware that interfaces between the navigation stack and SMACH (a library for creating state machines in ROS) using the MoveBaseClient. The MoveBaseClient serves as a bridge between your state machine's high-level actions and the low-level navigation functionality provided by the navigation stack in ROS. By completing this tutorial, you will understand how to integrate ROS navigation capabilities with state machines for autonomous robot navigation tasks.


> ## Main Goals
> - Understand the role of the navigation stack in ROS.
>- Learn how to utilize the MoveBaseClient to send navigation goals.
>- Implement a middleware that allows SMACH to control robot navigation using the MoveBaseClient.

## Background: Navigation Stack and MoveBaseClient
The navigation stack in ROS is a collection of packages that enable a robot to autonomously navigate through an environment. It consists of various components such as localization, mapping, and path planning. The MoveBaseClient is an action client that communicates with the move_base node, which is the core of the navigation stack. It sends navigation goals to the move_base node, which then plans and executes the robot's movement to reach the goal.


[![IMAGE ALT TEXT](https://i.ytimg.com/an_webp/1aX7NFvKehs/mqdefault_6s.webp?du=3000&sqp=CKvI4a8G&rs=AOn4CLBpK6JU4tyfNnHX5WQW1kiGh-h9Hg)](https://www.youtube.com/watch?v=1aX7NFvKehs "ROS Navigation stack Architecture in 4 minutes || A to Z Basics")


## Code Overview
We have provided a Python script with a basic implementation of the MoveBase class, which includes functionality for setting navigation goals and sending them using the MoveBaseClient. Your task is to complete certain parts of the code to make it fully functional.

### Code Snippet:
```python
# Import necessary ROS and actionlib libraries
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class MoveBase(object):
    def __init__(self):
        # Initialize variables and subscribers
        self._target_pose = self
        self._robot_pose  = self
        self._mba_client  = None
        self.odom_topic = '/amcl_pose'
        self.cmd_vel_topic = '/p3dx/cmd_vel'
        self._odom_sub    = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self.odom_cb)
        self._cmd_vel_pub = None
        rospy.sleep(.5)
        self.check()
    
    # Other methods...
```

## Instructions
1. **Understanding the Code**: Before proceeding, make sure you understand the provided code structure and its purpose. Familiarize yourself with the functions and variables defined in the MoveBase class.

2. **Completing the Middleware Functionality**:
   - In the `go()` method, complete the implementation to send the navigation goal to the move_base node using the MoveBaseClient. Use the `_call_mba_client()` method to send the goal.
   - Implement the `get_robot_pose()` method to retrieve the current robot's pose (position and orientation) from the odometry data.
   - Update the `get_target_pose()` method to return the target pose as a numpy array.
   - Complete the `_yaw_from_quat()` method to calculate the yaw angle (rotation around the z-axis) from the given quaternion orientation.

3. **Testing**:
   - Run the provided script after completing the necessary parts.
   - Test the functionality by setting navigation goals using the `rotate()` method or by setting custom target poses and calling the `go()` method.
   - Verify that the robot moves according to the specified goals.

## Summary
In this tutorial, you learned about the navigation stack in ROS and its role in enabling autonomous robot navigation. You also implemented a middleware using the MoveBaseClient to interface between the navigation stack and SMACH, allowing high-level control of robot navigation tasks. By completing this tutorial, you have gained valuable experience in integrating ROS navigation capabilities with state machines for building complex robot behaviors.