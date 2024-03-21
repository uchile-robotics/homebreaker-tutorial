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

        self.odom_topic = '/amcl_pose'
        self.cmd_vel_topic = '/cmd_vel'
        
        self._odom_sub    = rospy.Subscriber(self.odom_topic, PoseWithCovarianceStamped, self.odom_cb)
        self._cmd_vel_pub = None

        rospy.sleep(.5)
        self.check()
    
    def check(self, timeout=1.0):
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
        self._robot_pose.x = msg.pose.pose.position.x
        self._robot_pose.y = msg.pose.pose.position.y
        self._robot_pose.theta = self._yaw_from_quat(msg.pose.pose.orientation)

    def set_target(self, x, y, theta):
        self._target_pose.x = x
        self._target_pose.y = y
        self._target_pose.theta = theta

    def get_target_pose(self):
        np.array([self._target_pose.x, self._target_pose.y, self._target_pose.theta])

    def get_robot_pose(self):
        pass

    def go_to(self):
        try:
            result = self._call_mba_client()
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def rotate(self, theta):
        x = self._robot_pose.x
        y = self._robot_pose.y
        self.set_target(x, y, theta)
        self.go()

    def _call_mba_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self._target_pose.x
        goal.target_pose.pose.position.y = self._target_pose.y

        goal.target_pose.pose.orientation = self._euler_to_quat()

        self._mba_client.send_goal(goal)

        wait = self._mba_client.wait_for_result()

        if not wait:
            rospy.logerr("server not available!")
        else:
            return self._mba_client.get_result()

    def _euler_to_quat(self):
        q = quaternion_from_euler(0, 0, self._target_pose.theta/180.0*np.pi, 'rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    @staticmethod
    def _yaw_from_quat(q):
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

if __name__ == "__main__":
    rospy.init_node('move_base_client_test')
    mb = BaseSkill()
    mb.set_target(1, 5, 0)
    mb.go()
    mb.rotate(90)
    mb.rotate(0)