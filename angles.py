#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, String, int8, int32
from geometry_msgs.msg import Pose, Point, Quaternion
import math
#front=0
# right=1
# left=2
def inverse_kinematics(x,y):
    l1_offset=0
    l2_offset=0
    l1=23-l1_offset
    l2=19.6-l2_offset
    theta2=math.acos((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
    theta1=math.atan(y/x)-math.atan((l2*math.sin(theta2))/(l1+l2*math.cos(theta2)))
    return theta1, theta2
class MyNode(object):
    def init(self):
        rospy.init_node('my_node')
        self.subscription = rospy.Subscriber(
            't1',
            Int32MultiArray,
            self.t1_callback,
            queue_size=10
        )
        self.publisher = rospy.Publisher(
            't2',
            Int32MultiArray,
            queue_size=10
        )

    def t1_callback(self, msg):
        # Process the received Int64MultiArray message from t1

        # Call the controller function to handle the data
        theta1, theta2= inverse_kinematics(msg.data[0], msg.data[1])
        pose_msg = Pose()
        
        # Set the position (x, y, z)
        pose_msg.position = Point(x=theta1, y=theta2, z=0)
        
        # Set the orientation (quaternion: x, y, z, w)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Publish the Pose message
        self.publisher.publish(pose_msg)


def main():
    node = MyNode()
    rospy.spin()
if __name__== '__main__':
    main()