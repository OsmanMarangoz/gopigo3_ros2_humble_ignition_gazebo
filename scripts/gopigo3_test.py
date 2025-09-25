#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class gopigo3_robotcontroller(Node):

    def __init__(self):
        super().__init__('easygopigo3_ros2_driver')
        self.publisher_ = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    def _publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

    def forward(self):
        self._publish_twist(linear=1.0, angular=0.0)

    def stop(self):
        self._publish_twist(linear=0.0, angular=0.0)

    def left(self):
        self._publish_twist(linear=0.0, angular=1.0) 

    def right(self):
        self._publish_twist(linear=0.0, angular=-0.5) # -0.5 rad/s
        
    def backward(self):
        self._publish_twist(linear=-0.1, angular=0.0) # -0.1 m/s


def main(args=None):
    rclpy.init(args=args)
    gpg = gopigo3_robotcontroller()

    gpg.forward()
    time.sleep(5)
    gpg.stop()
    time.sleep(2)
    gpg.left()
    time.sleep(5)
    gpg.stop()
    gpg.forward()
    time.sleep(5)
    gpg.stop()

if __name__ == '__main__':
    main()