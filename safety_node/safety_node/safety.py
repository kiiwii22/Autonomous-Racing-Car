#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('Safety_Node')
        self.Lidar_data=self.create_subscription(LaserScan,"/scan",self.scan_callback,10)
        self.Odometry_data=self.create_subscription(Odometry,"/ego_racecar/odom",self.odom_callback,10)
        self.controller=self.create_publisher(AckermannDriveStamped,"/drive",1000)
        self.teleop_publisher_ = self.create_publisher(AckermannDriveStamped, 'teleop', 1000)
        self.speed_x=0.0

    def odom_callback(self,odom_msg: Odometry):
        self.speed_x=odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg: LaserScan):
        command= AckermannDriveStamped()
        self.EmergencyBraking=False
        TTC_threshold=2 #to be tuned
        for index, rg in enumerate(scan_msg.ranges):
            if (np.isnan(rg) or rg > scan_msg.range_max or rg <scan_msg.range_min):
                continue
            #calculate range_rate
            self.theta=scan_msg.angle_min+index*scan_msg.angle_increment
            self.range_rate=self.speed_x*np.cos(self.theta)

            self.TTC=rg/(max(self.range_rate, 0.001))
            if self.TTC<TTC_threshold:
                self.EmergencyBraking=True
                break

        if self.EmergencyBraking:
            command.drive.speed=0.0
            self.controller.publish(command)
            self.teleop_publisher_.publish(command)



def main(args=None):
    rclpy.init(args=args)
    safety_node=SafetyNode()
    rclpy.spin(safety_node)

    rclpy.shutdown()

    if __name__ == '__main__':
        main()