#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32
from funct_fin import steer_angle, rolling_friction, motor_force, F_friction_due_to_steering, slip_angles, lateral_tire_forces, friction
from discretization_function import compute_discrete_function_terms_single_step_euler
from continuous_matrix_function import continuous_matrices


class ViconAndSensorListener:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('vicon_and_sensor_listener', anonymous=True)

        # Initialize placeholders for the messages
        self.pose_data = None
        self.sensor_data = None
        self.vy_data = None

        # Subscribe to the topics
        rospy.Subscriber('/vicon/jetracer1', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/sensors_and_input_1', Float32MultiArray, self.sensor_callback)
        rospy.Subscriber('/vy_1', Float32, self.vy_callback)

        # Start the ROS loop
        rospy.spin()

    def pose_callback(self, geom):
        """Callback for PoseWithCovarianceStamped messages."""
        self.pose_data = geom
        self.combine_and_log()

    def sensor_callback(self, data):
        """Callback for Float32MultiArray messages."""
        self.sensor_data = data
        self.combine_and_log()

    def vy_callback(self, data):
        """Callback for vy messages"""
        self.vy_data = data.data
        self.combine_and_log()

    def combine_and_log(self):
        """Combine the pose and sensor data and log them."""
        if self.pose_data is None or self.sensor_data is None:
            # Wait until both messages are received
            return

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
        z = np.array([[vx], [vy], [w]])
        u = np.array([th, delta])
        # Construct the log message
        message = (
            "\n-------------------\n"
            f"x : {x:.2f} m\n"
            f"y : {y:.2f} m\n"
            f"yaw : {yaw:.2f} rad\n"
            f"v_x: {vx:.2f} m/s\n"
            f"v_y: {vy:.2f} m/s\n"
            f"Omega: {w:.2f} rad/s\n"
            f"Throttle: {th:.2f}\n"
            f"Steering: {steering_input:.2f}\n"
            "-------------------\n"
        )

        # Log the message
        rospy.loginfo(message)
        


if __name__ == '__main__':
    try:
        ViconAndSensorListener()
    except rospy.ROSInterruptException:
        pass
### OK ####