#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher:
    def __init__(self):
        # Internal variables
        self.rate = ...

        # Publishers
        self.pub = ...

    def run(self):
        while not rospy.is_shutdown():
            ...

if __name__ == '__main__':
    ...