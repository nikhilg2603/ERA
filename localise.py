#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, String, int8, int32
c1 = [0, 0]
c2 = [0, 0]
c3 = [0, 0]
target = 5
c = 0
#front=0
# right=1
# left=2

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
            String,
            queue_size=10
        )

    def t1_callback(self, msg):
        # Process the received Int64MultiArray message from t1
        received_data = msg.data
        x, y, distance, id = received_data

        # Call the controller function to handle the data
        self.controller(x, y, distance, id)

    def controller(self, x, y, distance, id):
        delta = 5  # threshold
        target_threshold = 2

        global c

        if x in range(c1[0] - delta, c1[0] + delta) and y in range(c1[1] - delta, c1[1] + delta):
            self.publish_to_output_topic(1)
            c = 1

        if x in range(c2[0] - delta, c2[0] + delta) and y in range(c2[1] - delta, c2[1] + delta):
            self.publish_to_output_topic(2)
            c = 2

        if distance <= target_threshold and id == target:
            c = 3

            # stop operating this node

        if c == 0:
            self.publish_to_output_topic(0)

    def publish_to_output_topic(self, direction):
        msg = String()
        msg.data = direction
        self.publisher.publish(msg)


def main():
    node = MyNode()
    rospy.spin()
if __name__== '__main__':
    main()