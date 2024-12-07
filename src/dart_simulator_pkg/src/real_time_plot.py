#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class RealTimePlotter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('real_time_plotter', anonymous=True)

        # Initialize data buffers
        self.time_data = deque(maxlen=100)  # Time buffer
        self.x_data = deque(maxlen=100)
        self.y_data = deque(maxlen=100)
        self.yaw_data = deque(maxlen=100)
        self.vx_data = deque(maxlen=100)
        self.vy_data = deque(maxlen=100)
        self.w_data = deque(maxlen=100)

        # Subscribe to the topics
        rospy.Subscriber('/vicon/jetracer1', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/sensors_and_input_1', Float32MultiArray, self.sensor_callback)

        # Set up the figure and subplots
        self.fig = plt.figure(figsize=(10, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2, 1])

        # First subplot: x vs y (spans two columns)
        self.ax_trajectory = self.fig.add_subplot(gs[0, :])
        self.ax_trajectory.set_title("Trajectory: x vs y")
        self.ax_trajectory.set_xlabel("x (m)")
        self.ax_trajectory.set_ylabel("y (m)")
        self.ax_trajectory.grid()

        # Second subplot: yaw vs time
        self.ax_yaw = self.fig.add_subplot(gs[1, 0])
        self.ax_yaw.set_title("Yaw vs Time")
        self.ax_yaw.set_xlabel("Time (s)")
        self.ax_yaw.set_ylabel("Yaw (rad)")
        self.ax_yaw.grid()

        # Third subplot: vx vs time
        self.ax_vx = self.fig.add_subplot(gs[1, 1])
        self.ax_vx.set_title("Vx vs Time")
        self.ax_vx.set_xlabel("Time (s)")
        self.ax_vx.set_ylabel("Vx (m/s)")
        self.ax_vx.grid()

        # Fourth subplot: vy vs time
        self.ax_vy = self.fig.add_subplot(gs[2, 0])
        self.ax_vy.set_title("Vy vs Time")
        self.ax_vy.set_xlabel("Time (s)")
        self.ax_vy.set_ylabel("Vy (m/s)")
        self.ax_vy.grid()

        # Fifth subplot: w vs time
        self.ax_w = self.fig.add_subplot(gs[2, 1])
        self.ax_w.set_title("Angular Velocity (w) vs Time")
        self.ax_w.set_xlabel("Time (s)")
        self.ax_w.set_ylabel("w (rad/s)")
        self.ax_w.grid()

        # Set up animation
        self.animation = FuncAnimation(self.fig, self.update_plots, interval=100)
        plt.show()

    def pose_callback(self, msg):
        """Callback for PoseWithCovarianceStamped messages."""
        self.time_data.append(rospy.get_time())
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

        # Extract yaw from quaternion
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        self.yaw_data.append(yaw)

    def sensor_callback(self, msg):
        """Callback for Float32MultiArray messages."""
        # Extract relevant data
        self.vx_data.append(msg.data[0])  # Assuming vx is at index 0
        self.vy_data.append(msg.data[1])  # Assuming vy is at index 1
        self.w_data.append(msg.data[2])   # Assuming w is at index 2

    def update_plots(self, frame):
        """Update the subplots with the latest data."""
        if len(self.time_data) > 1:
            # Adjust lengths to match the shortest list
            min_len = min(len(self.time_data), len(self.vx_data), len(self.vy_data), len(self.w_data), len(self.yaw_data))
            time_data = list(self.time_data)[:min_len]
            x_data = list(self.x_data)[:min_len]
            y_data = list(self.y_data)[:min_len]
            vx_data = list(self.vx_data)[:min_len]
            vy_data = list(self.vy_data)[:min_len]
            w_data = list(self.w_data)[:min_len]
            yaw_data = list(self.yaw_data)[:min_len]

            # Update trajectory plot (x vs y)
            self.ax_trajectory.clear()
            self.ax_trajectory.plot(x_data, y_data, label="Trajectory", lw=2)
            self.ax_trajectory.set_title("Trajectory: x vs y")
            self.ax_trajectory.set_xlabel("x (m)")
            self.ax_trajectory.set_ylabel("y (m)")
            self.ax_trajectory.grid()
            self.ax_trajectory.legend()

            # Update yaw vs time
            self.ax_yaw.clear()
            self.ax_yaw.plot(time_data, yaw_data, label="Yaw", color='orange')
            self.ax_yaw.set_title("Yaw vs Time")
            self.ax_yaw.set_xlabel("Time (s)")
            self.ax_yaw.set_ylabel("Yaw (rad)")
            self.ax_yaw.grid()
            self.ax_yaw.legend()

            # Update vx vs time
            self.ax_vx.clear()
            self.ax_vx.plot(time_data, vx_data, label="Vx", color='blue')
            self.ax_vx.set_title("Vx vs Time")
            self.ax_vx.set_xlabel("Time (s)")
            self.ax_vx.set_ylabel("Vx (m/s)")
            self.ax_vx.grid()
            self.ax_vx.legend()

            # Update vy vs time
            self.ax_vy.clear()
            self.ax_vy.plot(time_data, vy_data, label="Vy", color='green')
            self.ax_vy.set_title("Vy vs Time")
            self.ax_vy.set_xlabel("Time (s)")
            self.ax_vy.set_ylabel("Vy (m/s)")
            self.ax_vy.grid()
            self.ax_vy.legend()

            # Update angular velocity (w) vs time
            self.ax_w.clear()
            self.ax_w.plot(time_data, w_data, label="Angular Velocity", color='red')
            self.ax_w.set_title("Angular Velocity (w) vs Time")
            self.ax_w.set_xlabel("Time (s)")
            self.ax_w.set_ylabel("w (rad/s)")
            self.ax_w.grid()
            self.ax_w.legend()

if __name__ == '__main__':
    try:
        RealTimePlotter()
    except rospy.ROSInterruptException:
        pass
