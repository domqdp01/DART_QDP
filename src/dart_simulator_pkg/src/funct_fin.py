
import sys
import rospy
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from scipy import integrate
import tf_conversions
from dynamic_reconfigure.server import Server
from dart_simulator_pkg.cfg import dart_simulator_guiConfig
from tf.transformations import quaternion_from_euler

# define dynamic models
l = 0.175
l_r = 0.54 * l
l_f = l - l_r
m = 1.67
m_front_wheel = 0.847
m_rear_wheel = 0.733
Cf = m_front_wheel / m
Cr = m_rear_wheel / m
Jz = 0.006513

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


# Function to calculate rolling friction
def motor_force(th,v):
    a =  28.887779235839844
    b =  5.986172199249268
    c =  -0.15045104920864105
    w = 0.5 * (np.tanh(100*(th+c))+1)
    Fm =  (a - v * b) * w * (th+c)
    return Fm

def rolling_friction(v):
    a =  1.7194761037826538
    b =  13.312559127807617
    c =  0.289848655462265
    Ff = - a * np.tanh(b  * v) - v * c
    return Ff

# def motor_force(th,v):
#     a =  28.887779235839844
#     b =  5.986172199249268
#     c =  -0.15045104920864105
#     w = 0.5 * (np.tanh(100*(th+c))+1)
#     Fm =  (a - v * b) * w * (th+c)
#     return Fm

def friction(v):
    a =  1.7194761037826538
    b =  13.312559127807617
    c =  0.289848655462265
    Ff = - a * np.tanh(b  * v) - v * c
    return Ff

# Function to compute additional friction due to steering
def F_friction_due_to_steering(steer_angle, vx):
    a, b, d, e = -0.1183, 5.9159, 0.2262, 0.7793
    friction_term = a + (b * steer_angle * np.tanh(30 * steer_angle))
    vx_term = - (0.5 + 0.5 * np.tanh(20 * (vx - 0.3))) * (e + d * (vx - 0.5))
    F_frict = friction_term * vx_term
    return F_frict


def slip_angles(vx,vy,w,steering_angle):
    # evaluate slip angles
    Vy_wheel_r = vy - l_r * w # lateral velocity of the rear wheel
    Vx_wheel_r = vx 
    Vx_correction_term_r = 0.1*np.exp(-100*Vx_wheel_r**2) # this avoids the vx term being 0 and having undefined angles for small velocities
    # note that it will be negligible outside the vx = [-0.2,0.2] m/s interval.
    Vx_wheel_r = Vx_wheel_r + Vx_correction_term_r
    alpha_r = - np.arctan(Vy_wheel_r/ Vx_wheel_r) / np.pi * 180  #converting alpha into degrees
                
    # front slip angle
    Vy_wheel_f = (vy + w * l_f) #* np.cos(steering_angle) - vx * np.sin(steering_angle)
    Vx_wheel_f =  vx
    Vx_correction_term_f = 0.1*np.exp(-100*Vx_wheel_f**2)
    Vx_wheel_f = Vx_wheel_f + Vx_correction_term_f
    alpha_f = -( -steering_angle + np.arctan2(Vy_wheel_f, Vx_wheel_f)) / np.pi * 180  #converting alpha into degrees
    return alpha_f, alpha_r

# def lateral_tire_forces(alpha_f,alpha_r):
#     #front tire Pacejka tire model
#     d =  2.9751534461975098
#     c =  0.6866822242736816
#     b =  0.29280123114585876
#     e =  -3.0720443725585938
#     #rear tire linear model
#     d_t_r, c_t_r, b_t_r = -0.8547, 0.9591, 11.5493
#     c_r = 0.38921865820884705

#     F_y_f = d * np.sin(c * np.arctan(b * alpha_f - e * (b * alpha_f -np.arctan(b * alpha_f))))
#     # F_y_r = d_t_r * np.sin(c_t_r * np.arctan(b_t_r * alpha_r ))
#     F_y_r = c_r * alpha_r
#     return F_y_f, F_y_r


def lateral_tire_forces(alpha_f,alpha_r):
    #front tire Pacejka tire model
    d =  2.9751534461975098
    c =  0.6866822242736816
    b =  0.29280123114585876
    e =  -3.0720443725585938
    #rear tire linear model
    c_r = 0.38921865820884705


    F_y_f = d * np.sin(c * np.arctan(b * alpha_f - e * (b * alpha_f -np.arctan(b * alpha_f))))
    F_y_r = c_r * alpha_r
    return F_y_f, F_y_r



