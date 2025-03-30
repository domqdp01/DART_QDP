#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque
import numpy as np

class Yderivative:
    def __init__(self, car_number):
        # Inizializza il nodo ROS
        self.car_number = car_number
        rospy.init_node('calcolo_derivata_y', anonymous=True)

        # Coda per memorizzare gli ultimi 2 valori di y e tempo
        self.y_values = deque(maxlen=2)  
        self.t_values = deque(maxlen=2)  

        # Variabili per memorizzare le velocità ricevute
        self.vy_value = None  
        self.vx_value = None  

        # Sottoscrizione ai topic
        rospy.Subscriber(f"/vicon/jetracer{car_number}", PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber(f"/vy_{car_number}", Float32, self.callback_velocity_y)
        rospy.Subscriber(f"/sensors_and_input_{car_number}", Float32MultiArray, self.callback_sensors_and_input)

        # Publisher per la derivata e per l'errore
        self.derivata_pub = rospy.Publisher("/derivata_y", Float32, queue_size=10)
        self.errore_pub = rospy.Publisher("/errore_vy", Float32, queue_size=10)

        rospy.loginfo("Nodo calcolo_derivata_y avviato!")
        rospy.spin()

    def callback_sensors_and_input(self, msg):
        """Callback per leggere il valore di vx dal topic Float32MultiArray"""
        if len(msg.data) > 0:  # Assicuriamoci che l'array non sia vuoto
            self.vx_value = msg.data[6]  # Assumiamo che `vx` sia il primo elemento
            self.steer_input = msg.data[-1]
            self.time = msg.data[0]
            # rospy.loginfo(f"Velocità X ricevuta: {self.vx_value:.4f} m/s")
        else:
            rospy.logwarn("Messaggio vuoto ricevuto su /sensors_and_input_X")

    def callback_velocity_y(self, vy):
        """Callback per la velocità lungo y"""
        self.vy_value = vy.data  # Aggiorna la variabile con il valore attuale di vy

    def odom_callback(self, msg):
        """Callback per la lettura dei dati del sensore"""
        # Memorizza il valore di y e il timestamp corretto
        pos_y = msg.pose.pose.position.y
        self.y_values.append(pos_y)
        self.t_values.append(self.time)

        # Calcoliamo la derivata con differenze finite centrali
        if len(self.y_values) == 2:
            dt = self.t_values[-1] - self.t_values[0]
            if dt > 0:
                y_derivative = (self.y_values[-1] - self.y_values[0]) / dt

                # rospy.loginfo("-----------------------------")
                # rospy.loginfo(f"Derivata calcolata: {y_derivative:.4f} m/s")
                # rospy.loginfo(f"Velocità di Rviz (Vy) = {self.vy_value}")
                # # rospy.loginfo(f"Velocità X = {self.vx_value}")
                # rospy.loginfo("-----------------------------")
                self.derivata_pub.publish(y_derivative)
                #vx * np.sin(x[2]) + vy * np.cos(x[2])
                self.delta = steer_angle(self.steer_input)
                self.vy_ev = self.vx_value * np.sin(self.delta) + y_derivative * np.cos(self.delta)
                rospy.loginfo("-----------------------------")
                rospy.loginfo(f"Derivata calcolata: {self.vy_ev:.4f} m/s")
                rospy.loginfo(f"Velocità di Rviz (Vy) = {self.vy_value}")
                # rospy.loginfo(f"Velocità X = {self.vx_value}")
                rospy.loginfo("-----------------------------")

                # Se abbiamo ricevuto vy, calcoliamo l'errore
                if self.vy_value is not None:
                    errore_vy = self.vy_value - y_derivative
                    self.errore_pub.publish(errore_vy)
            else:
                rospy.logwarn("⚠️ Timestamp non valido, salto il calcolo della derivata.")

def steer_angle(steering_command):
    a =  1.6379064321517944
    b =  0.3301370143890381
    c =  0.019644200801849365
    d =  0.37879398465156555
    e =  1.6578725576400757

    w = 0.5 * (np.tanh(30*(steering_command+c))+1)
    steering_angle1 = b * np.tanh(a * (steering_command + c)) 
    steering_angle2 = d * np.tanh(e * (steering_command + c))
    steering_angle = (w)*steering_angle1+(1-w)*steering_angle2 

    return steering_angle

if __name__ == "__main__":
    try:
        car_number = 1
        Yderivative(car_number=car_number)
    except rospy.ROSInterruptException:
        pass
