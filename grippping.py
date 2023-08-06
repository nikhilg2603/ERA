#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import math

theta1 = 0
theta2 = 0

def inverse_kinematics(x, y):
    l1_offset = 0
    l2_offset = 0
    l1 = 23 - l1_offset
    l2 = 19.6 - l2_offset
    theta2 = math.acos((x ** 2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    theta1 = math.atan(y / x) - math.atan((l2 * math.sin(theta2)) / (l1 + l2 * math.cos(theta2)))
    return theta1, theta2

class MinimalSubscriber:

    def _init_(self):
        rospy.init_node('minimal_subscriber_publisher')
        self.subscription = rospy.Subscriber('t1', String, self.listener_callback)
        self.publisher_ = rospy.Publisher('t2', String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def listener_callback(self, msg):
        x, y = msg.data.split(',')
        global theta1, theta2
        theta1, theta2 = inverse_kinematics(float(x), float(y))
        rospy.loginfo('I heard: "%s"' % msg.data)

    def timer_callback(self, event):
        msg = String()
        msg.data = str(theta1) + ', ' + str(theta2)
        self.publisher_.publish(msg)
        rospy.loginfo('Publishing: "%s"' % msg.data)

def main():
    minimal_subscriber_publisher = MinimalSubscriber()
    rospy.spin()

if __name__ == '_main_':
    main()