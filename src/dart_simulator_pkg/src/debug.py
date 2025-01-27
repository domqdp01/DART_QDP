#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray, Float32
from funct_fin import steer_angle
from continuos_matrix_online import continuous_matrices
from discretization_function_online import compute_discrete_function_terms_single_step_euler
from remove_duplicate import remove_duplicates
import threading

# Usa il backend TkAgg per una migliore gestione della finestra
matplotlib.use('TkAgg')


class SetMembershipOnline:
    def __init__(self):
        rospy.init_node('SME_online_plot', anonymous=True)
        self.rate = rospy.Rate(100)

        self.pose_data = None
        self.sensor_data = None
        self.vy_data = None

        self.z_minus1 = None
        self.u_minus1 = None
        self.A_i_minus1 = None
        self.b_i_minus1 = None

        self.mu_lock = threading.Lock()
        self.valid_mu = [0, 0]

        rospy.Subscriber('/vicon/jetracer1', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/sensors_and_input_1', Float32MultiArray, self.sensor_callback)
        rospy.Subscriber('/vy_1', Float32, self.vy_callback)

        # Avvio del thread di plotting separato
        self.plot_thread = threading.Thread(target=self.run_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()

        self.process_data()

    def pose_callback(self, geom):
        self.pose_data = geom

    def sensor_callback(self, data):
        self.sensor_data = data

    def vy_callback(self, data):
        self.vy_data = data.data

    def setup_plot(self):
        """Configura il grafico."""
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('')
        self.ax.set_ylabel('μ')
        self.ax.set_ylim(0.7, 1.3)
        self.ax.set_xlim(0, 9)
        self.ax.xaxis.set_visible(False)  # Nasconde l'intero asse x


        self.x_data = np.arange(0, 10, 0.1)
        self.y_min_data = np.zeros_like(self.x_data)
        self.y_max_data = np.zeros_like(self.x_data)

        self.y_min_plot, = self.ax.plot(self.x_data, self.y_min_data, 'g-')
        self.y_max_plot, = self.ax.plot(self.x_data, self.y_max_data, 'g-')

        self.fill_between = self.ax.fill_between(
            self.x_data, self.y_min_data, self.y_max_data, color='green', alpha=0.5, label='θ_k'
        )

        self.ax.legend()
        plt.ion()  # Modalità interattiva per evitare il blocco

    def run_plot(self):
        """Aggiorna il grafico in un thread separato."""
        self.setup_plot()
        while not rospy.is_shutdown():
            with self.mu_lock:
                mu_min, mu_max = self.valid_mu

            self.y_min_plot.set_ydata(np.full_like(self.x_data, mu_min))
            self.y_max_plot.set_ydata(np.full_like(self.x_data, mu_max))

            self.fill_between.remove()
            self.fill_between = self.ax.fill_between(
                self.x_data, mu_min, mu_max, color='green', alpha=0.5
            )

            self.ax.set_title(f'μ appartiene a: [{mu_min:.4f}, {mu_max:.4f}]')
            # self.ax.set_ylim(min(0, mu_min - 0.1), max(2, mu_max + 0.1))

            plt.pause(0.1)  # Mantieni aggiornato il grafico senza bloccare le altre finestre

    def process_data(self):
        """Elabora i dati in un loop ROS separato."""
        while not rospy.is_shutdown():
            if self.pose_data is None or self.sensor_data is None or self.vy_data is None:
                rospy.loginfo("Waiting for all sensor data...")
                self.rate.sleep()
                continue

            quaternion = (
                self.pose_data.pose.pose.orientation.x,
                self.pose_data.pose.pose.orientation.y,
                self.pose_data.pose.pose.orientation.z,
                self.pose_data.pose.pose.orientation.w
            )

            roll, pitch, yaw = euler_from_quaternion(quaternion)

            vx = self.sensor_data.data[6]
            vy = self.vy_data
            w = self.sensor_data.data[5]
            th = self.sensor_data.data[8]
            steering_input = self.sensor_data.data[9]

            delta = steer_angle(steering_input)

            z = np.array([[vx], [vy], [w]])
            u = np.array([[th], [delta]])

            if self.z_minus1 is None and self.u_minus1 is None:
                self.z_minus1 = z
                self.u_minus1 = u

            F_i, G_i = continuous_matrices(self.z_minus1, self.u_minus1)
            A_i, b_i = compute_discrete_function_terms_single_step_euler(self.z_minus1, F_i, G_i)

            if self.A_i_minus1 is None and self.b_i_minus1 is None:
                A = A_i
                b = b_i
            else:
                A = np.concatenate((A_i, self.A_i_minus1), axis=0)
                b = np.concatenate((b_i, self.b_i_minus1), axis=0)

            valid_mu = []
            valid_A = []
            valid_b = []
            mu_values = np.where(A != 0, b / A, np.inf)
            mu_i = b_i/A_i

            for idx, mu in enumerate(mu_values.flatten()):
                satisfy_all = True
                for j in range(len(A)):
                    if not (A[j] * mu <= b[j] + 1e-6):
                        satisfy_all = False
                        break

                if satisfy_all:
                    valid_A.append(A[idx])
                    valid_b.append(b[idx])
                    valid_mu.append(max(mu, 0))

            valid_mu = remove_duplicates(valid_mu, 1e-3)
            valid_mu = np.sort(valid_mu)

            if len(valid_mu) == 2:
                rospy.loginfo(f"μ ∈ [{valid_mu[0]:.4f}, {valid_mu[1]:.4f}] ")
            else:
                rospy.loginfo(f"μ = {valid_mu[0]:.4f}")

            with self.mu_lock:
                self.valid_mu = valid_mu if len(valid_mu) == 2 else [valid_mu[0], valid_mu[0]]

            if valid_A and valid_b:
                self.A_i_minus1 = np.vstack(valid_A)
                self.b_i_minus1 = np.vstack(valid_b)

            self.z_minus1 = z
            self.u_minus1 = u

            self.rate.sleep()


if __name__ == '__main__':
    try:
        SetMembershipOnline()
    except rospy.ROSInterruptException:
        pass
