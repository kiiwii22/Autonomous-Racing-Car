#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.LaserSubscriber=self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback,10 )
        self.DrivePublisher=self.create_publisher(AckermannDriveStamped, drive_topic, 1000)

        # TODO: set PID gains
        self.kp = 3
        self.kd = 0.1
        self.ki = 0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.start_t = -1.0
        self.curr_t = 0.0
        self.prev_t = 0.0

    def get_range(self, range_data: LaserScan, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        index=(np.abs(angle-range_data.angle_min)/range_data.angle_increment)
        i=int(index)
        range=range_data.ranges[i]
        if (np.isnan(range)):
            return range_data.range_max
        return range


    def get_error(self, range_data:LaserScan, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        L=1

        a=self.get_range(range_data,np.radians(-50))
        b=self.get_range(range_data,np.radians(-90)) # 0 degrees is in front of the car
        theta_radians=np.radians(40)       

        #determine the angle alpha between the car's x-axis and the right wall
        alpha=np.arctan((a*np.cos(theta_radians)-b)/(a*np.sin(theta_radians)))
        #determine the current distance
        Dt=b*np.cos(alpha)
        #determine the projected distance
        Dt1=Dt+L*np.sin(alpha)

        #determine the previous error
        self.prev_error=self.error
        #determine the current error
        self.error=dist-Dt
        #determine integral error
        self.integral=self.integral+self.error

        self.prev_t=self.curr_t
        self.curr_t = range_data.header.stamp.nanosec * 10e-9 + range_data.header.stamp.sec
        if (self.start_t == 0.0):
            self.start_t = self.curr_t


    def pid_control(self):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        drive_msg = AckermannDriveStamped()
        # TODO: Use kp, ki & kd to implement a PID controller
        angle=self.kp*self.error+self.kd*(self.error-self.prev_error)/(self.curr_t-self.prev_t)
        

        # TODO: fill in drive message and publish

        drive_msg.drive.steering_angle=angle
        if (np.abs(angle)>=np.radians(0)) and (np.abs(angle< np.radians(10))):
            drive_msg.drive.speed=1.5
        elif ((np.abs(angle)>=np.radians(10)) and (np.abs(angle)<np.radians(20))):
            drive_msg.drive.speed=1.0
        else:
            drive_msg.drive.speed=0.5
        k=drive_msg.drive.speed

        self.DrivePublisher.publish(drive_msg)




    def scan_callback(self, msg:LaserScan):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        
        self.get_error(msg,1) # TODO: replace with error calculated by get_error()
        self.pid_control() # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized ")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()