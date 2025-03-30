#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class RvizListener:
    
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rviz_listener_2', anonymous=True)

        # Variables to store incoming data
        self.rviz_data = None
        self.vx_2 = None
        self.vy_2 = None
        self.omega_2 = None
        
        # Subscribers for the topics
        rospy.Subscriber('/rviz_data_2', PoseStamped, self.rviz_callback)
        rospy.Subscriber('/vx_2', Float32, self.vx_callback)
        rospy.Subscriber('/vy_2', Float32, self.vy_callback)
        rospy.Subscriber('/omega_2', Float32, self.omega_callback)

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
        self.vx_2 = msg.data
        self.log_data()

    def vy_callback(self, msg):
        # Save the linear velocity in the y direction
        self.vy_2 = msg.data
        self.log_data()

    def omega_callback(self, msg):
        # Save the angular velocity
        self.omega_2 = msg.data
        self.log_data()

    def log_data(self):
        # Ensure all data is available before logging
        if self.rviz_data and self.vx_2 is not None and self.vy_2 is not None and self.omega_2 is not None:
            rospy.loginfo(
                "\n-------------------\n"
                f"x      : {self.rviz_data['x']:.6f} m\n"
                f"y      : {self.rviz_data['y']:.6f} m\n"
                f"yaw    : {self.rviz_data['yaw']:.6f} rad\n"
                f"vx     : {self.vx_2:.6f} m/s\n"
                f"vy     : {self.vy_2:.6f} m/s\n"
                f"omega  : {self.omega_2:.6f} rad/s\n"
                "-------------------"
            )


if __name__ == '__main__':
    try:
        # Create an instance of the RvizListener class and keep the node running
        RvizListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
