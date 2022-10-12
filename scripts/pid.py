#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

right_laser = float()
left_laser = float()
front_laser = float()

odom = float()

class Controller:
    def __init__(self):
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.velocity = Twist()

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
            rospy.sleep(1)
            self.velocity.angular.z = 90*(math.pi/180)
            self.vel_pub.publish(self.velocity)
            rospy.sleep(1)
            self.velocity.angular.z = 0*(math.pi/180)
            self.vel_pub.publish(self.velocity)
            break

    def pid(self, error, gains = [0,0,0]):
        P = - gains[0] * error

        if P < -0.1:
            P = -0.1
        elif P > 0.1:
            P = 0.1
        
        return P
        

    def straight(self):
        error = left_laser - 0.03
        self.velocity.linear.x = 0.1
        # self.velocity.angular.z = self.pid(error, [0.005,0,0])
        self.vel_pub.publish(self.velocity)


def rightCallback(msg):
    global right_laser
    right_laser = msg.range

def leftCallback(msg):
    global left_laser
    left_laser = msg.range

def frontCallback(msg):
    global front_laser
    front_laser = msg.range

def odom_callback(msg):
    global odom
    odom = msg


if __name__ == "__main__":
    rospy.init_node("pid")
    rospy.loginfo("Starting pid.")
    rospy.Subscriber("ir_right_link", Range, rightCallback)
    rospy.Subscriber("ir_left_link", Range, leftCallback)
    rospy.Subscriber("ir_front_link", Range, frontCallback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    controller = Controller()

    while not rospy.is_shutdown():
        # print(f"left_laser: {round(left_laser,4)}; right_laser:{round(right_laser,4)}; front_laser:{round(front_laser,4)}")
        
        controller.left()
        controller.straight()
        print(controller.velocity.angular)

