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
        self.sub = rospy.Subscriber("/joy", Joy, self.callback)
        self.amcl = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        # Publish control instructions to possible_cmd
        self.publi = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=8)
        self.twist = Twist()
        self.pose = PoseWithCovarianceStamped()
        self.poses = []

    def callback(self, msg):
        a = msg.buttons[0]
        b = msg.buttons[1]
        x = msg.buttons[2]
        l_analog = msg.axes[1]
        r_analog = msg.axes[3]

        self.twist.linear.x = 0.5*l_analog
        self.twist.angular.z = 0.3*r_analog
        
        if a == 1:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
        if b == 1:
            self.poses.append(self.pose)
            print(self.poses)
        
        if x == 1:
            rospack = rospkg.RosPack()
            pkgDir = rospack.get_path('homebreaker_tutorial')
            np.save(pkgDir + '/poses/poses.npy', self.poses)
            
        self.publi.publish(self.twist)
        
    def amcl_callback(self, msg):
        
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w
        
        self.pose = [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]
        
        # position = pose[0:2]
        # orientation = pose[3:6]
        
        pass

def main():
    rospy.init_node('test')  # Create and register the node!

    obj = Joy_Template('args')  # Create an object of type Joy_Template, defined above

    rospy.spin()  # ROS function that prevents the program from ending - must be used with Subscribers

if __name__ == '__main__':
    main()
