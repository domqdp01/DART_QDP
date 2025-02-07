#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32
from funct_fin import steer_angle
from continuos_matrix_online_2D import continuous_matrices_2D
from discretization_function_online_2D import compute_discrete_function_terms_single_step_euler
from remove_duplicate import remove_duplicates
from functions_for_2D import compute_vertices, underapproximate_convex_polytope
import matplotlib.pyplot as plt
import threading
import matplotlib

matplotlib.use('TkAgg')

class SetMermbershipOnline:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('SME_online_plot_2D', anonymous=True)

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

        self.plot_thread = threading.Thread(target=self.run_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()

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
            
            direction = np.array([
                [1., 0.],   
                [-1., 0.],  
                [0., 1.],   
                [0., -1.]   
            ])
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
            F_i, G_i = continuous_matrices_2D(self.z_minus1, self.u_minus1)
            
            ## COMPUTE THE AUTONOUS AND INPUT MATRIX IN DISCRETE TIME
            A_i, b_i = compute_discrete_function_terms_single_step_euler(self.z_minus1, F_i, G_i)

            # rospy.loginfo(f"A_i = {A_i}\n"
            #               f"b_i = {b_i}")
            if self.A_i_minus1 is None and self.b_i_minus1 is None:
                A = A_i
                b = b_i

            else:
                A = np.concatenate((A_i, self.A_i_minus1), axis=0)
                b = np.concatenate((b_i, self.b_i_minus1), axis=0)

            # ### ===================================================== ###
            # ###             FEASIBLE PARAMETER SET      θ_K           ###
            # ### ===================================================== ###
            
            ## These regard A and b TOTAL
            vertex = compute_vertices(A, b)
            if len(vertex) == 0:
                # print(f"Warning: No vertex found at iteration {index}. Skipping this iteration.")
                continue  # Skip this iteration to avoid errors
            vertex = np.array(vertex)

            if vertex.shape[1] == 0:
                # print(f"⚠️  Iteration {index}: Vertex is empty (shape={vertex.shape}), skipping iteration.")
                continue

            if vertex.ndim > 2:
                vertex = vertex.squeeze(-1)
            # print(f"Debug: vertex.shape = {vertex.shape}")

            Hp, hp = underapproximate_convex_polytope(vertex, direction)

            # These regard A_i and b_i ACTUAL ITERATION
            vertex_i = compute_vertices(A_i, b_i)
            if len(vertex_i) == 0:
                # print(f"Warning: No vertex found at iteration {index}. Skipping this iteration.")
                continue  # Skip this iteration to avoid errors
            vertex_i = np.array(vertex_i)

            if vertex_i.shape[1] == 0:
                # print(f"⚠️  Iteration {index}: Vertex is empty (shape={vertex.shape}), skipping iteration.")
                continue

            if vertex_i.ndim > 2:
                vertex_i = vertex_i.squeeze(-1)
            # print(f"Debug: vertex.shape = {vertex.shape}")

            Hp_i, hp_i = underapproximate_convex_polytope(vertex_i, direction)

            # rospy.loginfo(f"Hp = {Hp}\n"
            #               f"hp = {hp}")
            
            # Evaluating FINAL MU
            mu_1_up = mu_1_low = mu_2_up = mu_2_low = None  

            if Hp.shape[0] > 0 and hp.shape[0] > 0:
                mu_1_up = hp[0] / Hp[0, 0] if Hp[0, 0] != 0 else None  # Evita inf

            if Hp.shape[0] > 1 and hp.shape[0] > 1:
                mu_1_low = hp[1] / Hp[1, 0] if Hp[1, 0] != 0 else None  # Evita inf

            if Hp.shape[0] > 2 and hp.shape[0] > 2:
                mu_2_up = hp[2] / Hp[2, 1] if Hp[2, 1] != 0 else None  # Evita inf

            if Hp.shape[0] > 3 and hp.shape[0] > 3:
                mu_2_low = hp[3] / Hp[3, 1] if Hp[3, 1] != 0 else None  # Evita None

            print(f"mu_1 = [{mu_1_low}, {mu_1_up}], mu_2 = [{mu_2_low}, {mu_2_up}]")

            # Evaluanting CURRENT MU
            mu_i_1_up = mu_i_1_low = mu_i_2_up = mu_i_2_low = None  

            if Hp_i.shape[0] > 0 and hp_i.shape[0] > 0:
                mu_i_1_up = hp_i[0] / Hp_i[0, 0] if Hp_i[0, 0] != 0 else None  # Evita inf

            if Hp_i.shape[0] > 1 and hp_i.shape[0] > 1:
                mu_i_1_low = hp_i[1] / Hp_i[1, 0] if Hp_i[1, 0] != 0 else None  # Evita inf

            if Hp_i.shape[0] > 2 and hp_i.shape[0] > 2:
                mu_i_2_up = hp_i[2] / Hp_i[2, 1] if Hp_i[2, 1] != 0 else None  # Evita inf

            if Hp_i.shape[0] > 3 and hp_i.shape[0] > 3:
                mu_i_2_low = hp_i[3] / Hp_i[3, 1] if Hp_i[3, 1] != 0 else None  # Evita Nones

            print(f"mu_i_1 = [{mu_i_1_low}, {mu_i_1_up}], mu_i_2 = [{mu_i_2_low}, {mu_i_2_up}]")



            self.A_i_minus1 = Hp
            self.b_i_minus1 = hp
            self.b_i_minus1 = np.atleast_2d(self.b_i_minus1).T
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