#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32

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

        # Construct the log message
        message = (
            "\n-------------------\n"
            f"x : {self.pose_data.pose.pose.position.x} m\n"
            f"y : {self.pose_data.pose.pose.position.y} m\n"
            f"yaw : {yaw} rad\n"
            f"v_x: {self.sensor_data.data[6]:.2f} m/s\n"
            f"v_y: {self.vy_data:.2f} m/s\n"
            f"Omega: {self.sensor_data.data[5]:.2f} rad/s\n"
            f"Throttle: {self.sensor_data.data[8]:.2f}\n"
            f"Steering: {self.sensor_data.data[9]:.2f}\n"
            "-------------------\n"
        )

        # Log the message
        rospy.loginfo(message)


if __name__ == '__main__':
    try:
        ViconAndSensorListener()
    except rospy.ROSInterruptException:
        pass
