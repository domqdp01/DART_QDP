#!/usr/bin/env python3

import rospy
import pygame
from std_msgs.msg import Float32

# Inizializza pygame per gestire la tastiera
pygame.init()
pygame.display.set_mode((100, 100))  # Necessario per catturare gli eventi della tastiera


def teleop_keyboard_v_ref():
    # Setup dei publisher ROS
    pub_steering_angle = rospy.Publisher('/steering_1', Float32, queue_size=8)
    pub_v_ref = rospy.Publisher('/vx_1', Float32, queue_size=8)
    pub_safety_value = rospy.Publisher('/safety_value', Float32, queue_size=8)

    rospy.init_node('teleop_keyboard_v_ref_sim', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Variabili di controllo
    v_ref = 0.0  # [m/s]
    incr = 0.1  # Incremento della velocità
    steering_angle = 0.0
    max_steering_rad = 0.3  # Valore massimo dell'angolo di sterzata

    while not rospy.is_shutdown():

        for event in pygame.event.get():  # Controllo eventi tastiera
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_u:  # Aumenta velocità
                    v_ref += incr
                    print(f'v_ref = {v_ref}')
                elif event.key == pygame.K_i:  # Diminuisce velocità
                    v_ref -= incr
                    print(f'v_ref = {v_ref}')
                elif event.key == pygame.K_a:  # Sterzata sinistra
                    steering_angle = -max_steering_rad
                elif event.key == pygame.K_d:  # Sterzata destra
                    steering_angle = max_steering_rad
                elif event.key == pygame.K_SPACE:  # Attiva la sicurezza
                    pub_safety_value.publish(1)
                else:
                    pub_safety_value.publish(0)

            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_a, pygame.K_d]:  # Rilascio del tasto sterzata
                    steering_angle = 0.0

        # Pubblicazione dei dati
        pub_v_ref.publish(v_ref)
        pub_steering_angle.publish(steering_angle)

        rate.sleep()


if __name__ == '__main__':
    try:
        teleop_keyboard_v_ref()
    except rospy.ROSInterruptException:
        pass
