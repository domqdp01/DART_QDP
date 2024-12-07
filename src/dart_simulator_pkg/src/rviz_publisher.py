#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class RvizListener:
    
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rviz_listener', anonymous=True)

        # Variables to store incoming data
        self.rviz_data = None
        self.vx_1 = None
        self.vy_1 = None
        self.omega_1 = None
        
        # Subscribers for the topics
        rospy.Subscriber('/rviz_data_1', PoseStamped, self.rviz_callback)
        rospy.Subscriber('/vx_1', Float32, self.vx_callback)
        rospy.Subscriber('/vy_1', Float32, self.vy_callback)
        rospy.Subscriber('/omega_1', Float32, self.omega_callback)

    def rviz_callback(self, msg):
        # Extract the orientation quaternion from the PoseStamped message
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        # Convert the quaternion to Euler angles (roll, pitch, yaw)
        _, _, yaw = euler_from_quaternion(quaternion)

        # Save position and yaw
        self.rviz_data = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": yaw
        }
        self.log_data()

    def vx_callback(self, msg):
        # Save the linear velocity in the x direction
        self.vx_1 = msg.data
        self.log_data()

    def vy_callback(self, msg):
        # Save the linear velocity in the y direction
        self.vy_1 = msg.data
        self.log_data()

    def omega_callback(self, msg):
        # Save the angular velocity
        self.omega_1 = msg.data
        self.log_data()

    def log_data(self):
        # Ensure all data is available before logging
        if self.rviz_data and self.vx_1 is not None and self.vy_1 is not None and self.omega_1 is not None:
            rospy.loginfo(
                "\n-------------------\n"
                f"x      : {self.rviz_data['x']:.6f} m\n"
                f"y      : {self.rviz_data['y']:.6f} m\n"
                f"yaw    : {self.rviz_data['yaw']:.6f} rad\n"
                f"vx     : {self.vx_1:.6f} m/s\n"
                f"vy     : {self.vy_1:.6f} m/s\n"
                f"omega  : {self.omega_1:.6f} rad/s\n"
                "-------------------"
            )


if __name__ == '__main__':
    try:
        # Create an instance of the RvizListener class and keep the node running
        RvizListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
