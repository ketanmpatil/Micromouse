#! /usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class Main():
    def __init__(self):
        rospy.init_node("wall-follow")
        self.range = None
        self.vel = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
        self.velocity = Twist()
        self.range_sub = rospy.Subscriber("ir_left_link", Range, self.range_callback) 
        self.count = 0
    def range_callback(self, msg):
        if msg.range < 0.04:
            self.range = 0.02
        else:
            self.range = msg.range
        

    def follow(self):
            try:
                print(self.range)
                self.velocity.linear.x = 0.1
                self.velocity.angular.z = 0
                self.vel.publish(self.velocity)

                if self.range > 0.1 and self.range < 0.4:
                    self.velocity.angular.z = 0.8
                    self.vel.publish(self.velocity)
                
                elif self.range < 0.1:
                    self.velocity.angular.z = -0.8
                    self.vel.publish(self.velocity)

                elif self.range > 0.4:
                    self.leftTurn()
                    
            except:
                if self.count == 0:
                    print("Waiting for the range..", end="")
                    self.count = 1
                print(".",end="")

    def leftTurn(self):
        t = time.time()
        rospy.sleep(0.1)
        while time.time() - t < 0.7:
            print(time.time() - t)
            self.velocity.linear.x = 0
            self.velocity.angular.z = 6
            self.vel.publish(self.velocity)
                    
        self.velocity.angular.z = 0
        self.vel.publish(self.velocity)

    def rightTurn(self):
        t = time.time()
        rospy.sleep(0.2)
        while time.time() - t < 0.7:
            print(time.time() - t)
            self.velocity.linear.x = 0
            self.velocity.angular.z = 6
            self.vel.publish(self.velocity)
                    
        self.velocity.angular.z = 0
        self.vel.publish(self.velocity)



            
if __name__ == "__main__":
    wallFollow = Main()
    while not rospy.is_shutdown():
        wallFollow.follow()
        # wallFollow.rightTurn()
