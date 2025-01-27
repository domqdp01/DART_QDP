#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32
from funct_fin import steer_angle
from continuos_matrix_online import continuous_matrices
from discretization_function_online import compute_discrete_function_terms_single_step_euler
from remove_duplicate import remove_duplicates


class SetMermbershipOnline:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('Set_membership_estimation_online', anonymous=True)

        # Set processing rate to 100 Hz (adjustable)
        self.rate = rospy.Rate(100)  # Process data every 0.01 seconds

        # Initialize placeholders for the messages
        self.pose_data = None
        self.sensor_data = None
        self.vy_data = None

        # Variables to store current and previous values of z and u
        self.z_prev = None
        self.u_prev = None

        self.z_minus1 = None
        self.u_minus1 = None

        self.A_i_minus1 = None
        self.b_i_minus1 = None

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

            ### ===================SetMermbershipOnline:
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

            # Store current states and inputs
            z = np.array([[vx], [vy], [w]])
            u = np.array([[th], [delta]])

            # Initialization of the vectors
            if self.z_minus1 is None and self.u_minus1 is None:
                self.z_minus1 = z  
                self.u_minus1 = u

            ### ===================================================== ###
            ###           UNFALISIFIED PARAMETER SET    Δ_K           ###
            ### ===================================================== ###


            ## COMPUTE THE AUTONOUS AND INPUT MATRIX IN CONTINUOUS TIME
            F_i, G_i = continuous_matrices(self.z_minus1, self.u_minus1)
            
            ## COMPUTE THE AUTONOUS AND INPUT MATRIX IN DISCRETE TIME
            A_i, b_i = compute_discrete_function_terms_single_step_euler(self.z_minus1, F_i, G_i)

            if self.A_i_minus1 is None and self.b_i_minus1 is None:
                A = A_i
                b = b_i

            else:
                A = np.concatenate((A_i, self.A_i_minus1), axis=0)
                b = np.concatenate((b_i, self.b_i_minus1), axis=0)

            ### ===================================================== ###
            ###             FEASIBLE PARAMETER SET      θ_K           ###
            ### ===================================================== ###
            
            valid_mu = []
            valid_A = []
            valid_b = []
            mu_values = np.where(A != 0, b / A, np.inf)  # Skip division where A = 0

            for idx, mu in enumerate(mu_values.flatten()):
                satisfy_all = True
                for j in range(len(A)):
                    if not (A[j] * mu <= b[j] + 1e-6):
                        satisfy_all = False
                        break
                
                if satisfy_all:
                    valid_A.append(A[idx])
                    valid_b.append(b[idx])
                    valid_mu.append(max(mu, 0))  # Ensure non-negative mu

            valid_mu = remove_duplicates(valid_mu, 1e-3)
            valid_mu = np.sort(valid_mu)
            
            if valid_mu.shape == 2:
                rospy.loginfo(f"μ ∈ [{valid_mu[0]:.4f}, {valid_mu[1]:.4f}] ")
            else:
                rospy.loginfo(f"μ = {valid_mu}") 

            if valid_A and valid_b:
                self.A_i_minus1 = np.vstack(valid_A)
                self.b_i_minus1 = np.vstack(valid_b)
                      

            
            # uptade values
            self.z_minus1 = z
            self.u_minus1 = u           
            # Sleep to maintain the desired processing frequency
            self.rate.sleep()

if __name__ == '__main__':
    try:
        SetMermbershipOnline()
    except rospy.ROSInterruptException:
        pass

## 12:19 27/01/2025