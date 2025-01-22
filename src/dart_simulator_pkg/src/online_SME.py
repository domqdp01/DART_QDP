#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32
from funct_fin import steer_angle

class ViconAndSensorListener:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('vicon_and_sensor_listener', anonymous=True)

        # Set processing rate to 10 Hz (adjustable)
        self.rate = rospy.Rate(10)  # Process data every 0.1 seconds

        # Initialize placeholders for the messages
        self.pose_data = None
        self.sensor_data = None
        self.vy_data = None

        # Variables to store current and previous values of z and u
        self.z_prev = None
        self.u_prev = None

        # Subscribe to the topics
        rospy.Subscriber('/vicon/jetracer1', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/sensors_and_input_1', Float32MultiArray, self.sensor_callback)
        rospy.Subscriber('/vy_1', Float32, self.vy_callback)

        # Start processing loop
        self.process_data()

    def pose_callback(self, geom):
        """Callback for PoseWithCovarianceStamped messages."""
        self.pose_data = geom

    def sensor_callback(self, data):
        """Callback for Float32MultiArray messages."""
        self.sensor_data = data

    def vy_callback(self, data):
        """Callback for vy messages"""
        self.vy_data = data.data

    def process_data(self):
        """Main processing loop running at a controlled rate."""
        while not rospy.is_shutdown():
            if self.pose_data is None or self.sensor_data is None or self.vy_data is None:
                rospy.loginfo("Waiting for all sensor data...")
                self.rate.sleep()
                continue

            # Extract the quaternion from the pose data
            quaternion = (
                self.pose_data.pose.pose.orientation.x,
                self.pose_data.pose.pose.orientation.y,
                self.pose_data.pose.pose.orientation.z,
                self.pose_data.pose.pose.orientation.w
            )

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion(quaternion)

            x = self.pose_data.pose.pose.position.x
            y = self.pose_data.pose.pose.position.y
            vx = self.sensor_data.data[6]
            vy = self.vy_data
            w = self.sensor_data.data[5]
            th = self.sensor_data.data[8]
            steering_input = self.sensor_data.data[9]

            delta = steer_angle(steering_input)

            # Store current values
            z_new = np.array([[vx], [vy], [w]])
            u_new = np.array([th, delta])

            # Print current and previous values (before updating)
            message = (
                "\n-------------------\n"
                f"Previous z: {self.z_prev.flatten().tolist() if self.z_prev is not None else 'None'}\n"
                f"Previous u: {self.u_prev.tolist() if self.u_prev is not None else 'None'}\n"
                f"Current z: {z_new.flatten().tolist()}\n"
                f"Current u: {u_new.tolist()}\n"
                "-------------------\n"
            )
            rospy.loginfo(message)

            # Update previous values
            self.z_prev = z_new
            self.u_prev = u_new

            # Sleep to maintain the desired processing frequency
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ViconAndSensorListener()
    except rospy.ROSInterruptException:
        pass

## 20:06