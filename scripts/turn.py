#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

odom = int()

class Turn:
    def __init__(self):
        rospy.init_node("turn")
        # rospy.loginfo("Starting ROSNode as name_node.")
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.velocity = Twist()

    def odom_callback(self, msg):
        global odom
        odom = msg

    def right(self):
        while True:
            rospy.sleep(1)
            self.velocity.angular.z = 90*(math.pi/180)
            self.vel_pub.publish(self.velocity)
            rospy.sleep(1)
            self.velocity.angular.z = 0*(math.pi/180)
            self.vel_pub.publish(self.velocity)
            break

    def left(self):
        while True:
            self.velocity.angular.z = -90*(math.pi/180)
            self.vel_pub.publish(self.velocity)
            



if __name__ == "__main__":
    name_node = Turn()
    while not rospy.is_shutdown():
        print("Oh")
        Turn().right()
        rospy.sleep(2)
