#!/usr/bin/env python

import rospy  # Import ROS for Python
import rospkg
import numpy as np
from std_msgs.msg import String, Int32  # Import ROS messages of type String and Int32
from sensor_msgs.msg import Joy  # Import Joy message type
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped # Import ROS messages of type geometry


class Joy_Template(object):
    def __init__(self, args):
        super(Joy_Template, self).__init__()
        self.args = args

        # Subscribe to Joy messages
        self._joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        # Subscribe to AMCL pose messages
        self._amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        
        # velocity publisher
        self._cmd_vel_pub = rospy.Publisher("look-for-the-cmdvel-topic", Twist, queue_size=8)

        # ROS Messages
        self.twist = Twist()
        self.robot_pose = None
        self.poses = []

    def joy_callback(self, msg):
        """This method will be called everytime a new Joy message is received.

        Args:
            msg (sensor_msgs/Joy): The incoming joy message.
            definition at: https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html
        """

        # Choose which buttons to use
        a = msg.buttons[...] 
        b = msg.buttons[...] 
        x = msg.buttons[...] 
        l_analog = msg.axes[...]
        r_analog = msg.axes[...]

        # Choose a multiplier for the linear and angular speed
        # !Note: The velocity is in [m/s] and [rad/s]. Use multipliers that make sense
        # (the robot cannot muve faster than 1.2 [m/s])
        self.twist.linear.x  = ... * l_analog
        self.twist.angular.z = ... * r_analog
        
        # STOP Button
        if a == 1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        
        # Add pose button
        if b == 1:
            self.poses.append(self...)
        
        # Save ALL poses to file button
        if x == 1:
            rospack = rospkg.RosPack()
            pkgDir = rospack.get_path('homebreaker_tutorial')
            np.save(pkgDir + '/poses/poses.npy', self.poses)


        # Publish the velocity message    
        self._cmd_vel_pub.publish(self.twist)
        

    def amcl_callback(self, msg):
        """This method will be called everytime a new AMCL message is received.
            Saves the current pose reported by AMCL to the robot_pose variable.

        Args:
            msg (geometry_msgs/PoseWithCovarianceStamped): The incoming Pose message.
            definition at: https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
        """
        
        # Save the corresponding pose message components
        pos_x = msg. ...
        pos_y = msg. ...
        pos_z = msg. ...
        
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w
        
        self.robot_pose = [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]
        

def main():
    rospy.init_node('joy_test')  # Create and register the node!

    obj = Joy_Template('args')  # Create an object of type Joy_Template, defined above

    rospy.spin()  # ROS function that prevents the program from ending - must be used with Subscribers

if __name__ == '__main__':
    main()
