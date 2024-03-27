#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, PoseWithCovarianceStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

class BaseSkill(object):
    def __init__(self):
        self._target_pose = self
        self._robot_pose  = self
        self._mba_client  = None

        self.odom_topic = 'epic-odom-topic'
        self.cmd_vel_topic = 'epic-cmd-vel-topic'
        
        self._odom_sub    = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self.odom_cb)
        self._cmd_vel_pub = None

        rospy.sleep(.5)
        self.check()
    
    def check(self, timeout=1.0):
        """Checks if all the required services exists when starting the skill.

        Args:
            timeout (float, optional): Maximum time to wait for the services.

        Returns:
            bool: True if all the required services exist.
        """

        self._mba_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not self._mba_client.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr('Move base action server not found')
            rospy.signal_shutdown("server not available!")
            return False
        
        rospy.loginfo('Move base action server [OK]')

        try:
            rospy.wait_for_message(self.odom_topic, Odometry, timeout=timeout)
            rospy.loginfo('Odometry [OK]')
        except:
            rospy.logerr('Odometry not found')
            return False

        return True
    
    def odom_cb(self, msg):
        """ Updates the robot 2D pose register.
        """

        self._robot_pose.x = msg.pose.pose.position.x
        self._robot_pose.y = msg.pose.pose.position.y
        self._robot_pose.theta = self._yaw_from_quat(msg.pose.pose.orientation)

    def set_target(self, x, y, theta):
        """Writes on the target variables the incoming pose.

        Args:
            x (float): The target x position in [m]
            y (float): The target y position in [m]
            theta (float): The target rotation in [rad]
        """

        self._target_pose.x = x
        self._target_pose.y = y
        self._target_pose.theta = theta

    def get_target_pose(self):
        """ Returns the target pose.

        Returns:
            np.ndarray: The target pose (x, y, theta), with theta in degrees
        """

        return np.array([self._target_pose.x, self._target_pose.y, self._target_pose.theta])

    def get_robot_pose(self):
        """This function should return the current robot 2d pose.
        """

        pass

    def go(self):
        """Calls the Move Base Action server with the requested targte pose.
        """

        try:
            result = self._call_mba_client()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def rotate(self, theta):
        """ Rotates the robot by theta degrees

        Args:
            theta (float): The desired angle in degrees
        """

        ###
        ### Your logic here. Hint: the robot must maintain its x-y position
        ###

        self.go()

    def _call_mba_client(self):
        """ Makes a call to the Move Base Action server with the target pose
        and returns the result.
        """

        # Create a MoveBaseGoal message and assign the correspoinding target and 
        # make a call to the mba_client.
        ### Your code here



        ###

        wait = self._mba_client.wait_for_result()

        if not wait:
            rospy.logerr("server not available!")
        else:
            return self._mba_client.get_result()

    def _euler_to_quat(self):
        """ Convert euler angles to quaternion.
        Hint: search for 'quaternion_from_euler'
        """  
        q = ...
        return Quaternion(q[0], q[1], q[2], q[3])

    @staticmethod
    def _yaw_from_quat(q):
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


if __name__ == "__main__":

    # Run this script only to test if the skill works properly
    
    rospy.init_node('move_base_client_test')
    mb = BaseSkill()
    
    # You can set different goals to test the code.
    # Make sure the targets are feasible.
    # Example:
    mb.set_target(1, 1, 0) # go to position x: 1m, y: 1m, theta: 0 deg
    mb.go()
    mb.rotate(90) # rotate to angle 90 degrees
    mb.rotate(0)  # rotate back to zero