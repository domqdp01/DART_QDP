#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from scipy import integrate
import tf_conversions


# Definizione dei parametri del veicolo
l = 0.175
l_r = 0.54 * l
l_f = l - l_r
m = 1.67
Jz = 0.006513

def steer_angle(steering_command):
    a, b, c, d, e = 1.6379, 0.3301, 0.0196, 0.3788, 1.6579
    w = 0.5 * (np.tanh(30 * (steering_command + c)) + 1)
    steering_angle1 = b * np.tanh(a * (steering_command + c))
    steering_angle2 = d * np.tanh(e * (steering_command + c))
    return w * steering_angle1 + (1 - w) * steering_angle2

def motor_force(th, v):
    a, b, c = 28.8878, 5.9862, -0.1505
    w = 0.5 * (np.tanh(100 * (th + c)) + 1)
    return (a - v * b) * w * (th + c)

def friction(v):
    a, b, c = 1.7195, 13.3126, 0.2898
    return -a * np.tanh(b * v) - v * c

def kinematic_bicycle(t, z):
    u = z[0:2]
    x = z[2:]
    th, vx, vy, w = u[0], x[3], x[4], x[5]
    steering_angle = steer_angle(u[1])
    Fx = motor_force(th, vx) + friction(vx)
    acc_x = Fx / m
    w = vx * np.tan(steering_angle) / l
    vy = l_r * w
    xdot1 = vx * np.cos(x[2]) - vy * np.sin(x[2])
    xdot2 = vx * np.sin(x[2]) + vy * np.cos(x[2])
    xdot3 = w
    xdot4 = acc_x
    return np.array([0, 0, xdot1, xdot2, xdot3, xdot4, 0, 0])

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

def dynamic_bicycle(t,z):  # RK4 wants a function that takes as input time and state
    #z = throttle delta x y theta vx vy w
    u = z[0:2]
    x = z[2:]
    th = u[0]
    vx = x[3]
    vy = x[4]
    w = x[5]

    # evaluate steering angle 
    steering_angle = steer_angle(u[1])

    # evaluare slip angles
    alpha_f,alpha_r =slip_angles(vx,vy,w,steering_angle)

    #evaluate forward force
    Fx_wheels = motor_force(th,vx) + friction(vx)
    # assuming equally shared force among wheels
    Fx_f = Fx_wheels/2
    Fx_r = Fx_wheels/2

    # evaluate lateral tire forces
    F_y_f, F_y_r = lateral_tire_forces(alpha_f,alpha_r)

    # solve equations of motion for the rigid body
    A = np.array([[+np.cos(steering_angle),1,-np.sin(steering_angle),0],
                  [+np.sin(steering_angle),0, np.cos(steering_angle),1],
                  [l_f*np.sin(steering_angle),0             ,l_f     ,-l_r]])

    b = np.array([Fx_f,
                  Fx_r,
                  F_y_f,
                  F_y_r])

    [Fx, Fy, M] = A @ b

    acc_x =  Fx / m  + w * vy# acceleration in the longitudinal direction
    acc_y =  Fy / m  - w * vx# acceleration in the latera direction
    acc_w =  M / Jz # acceleration yaw


    xdot1 = vx * np.cos(x[2]) - vy * np.sin(x[2])
    xdot2 = vx * np.sin(x[2]) + vy * np.cos(x[2])
    xdot3 = w
    xdot4 = acc_x  
    xdot5 = acc_y  
    xdot6 = acc_w  

    # assemble derivatives [th, stter, x y theta vx vy omega], NOTE: # for RK4 you need to supply also the derivatives of the inputs (that are set to zero)
    zdot = np.array([0,0, xdot1, xdot2, xdot3, xdot4, xdot5, xdot6]) 
    return zdot

class Forward_intergrate_vehicle:
    def __init__(self, car_number, vehicle_model, initial_state, dt_int):
        self.safety_value = 0
        self.steering = 0
        self.throttle = 0
        self.state = initial_state
        self.dt_int = dt_int
        self.vehicle_model = vehicle_model
        self.car_number = car_number
        self.initial_time = rospy.Time.now()

        rospy.Subscriber(f'steering_{car_number}', Float32, self.callback_steering)
        rospy.Subscriber(f'throttle_{car_number}', Float32, self.callback_throttle)
        rospy.Subscriber('safety_value', Float32, self.callback_safety)
        self.pub_motion_capture_state = rospy.Publisher(f'vicon/jetracer{car_number}', PoseWithCovarianceStamped, queue_size=10)
        self.pub_sens_input = rospy.Publisher(f'sensors_and_input_{car_number}', Float32MultiArray, queue_size=1)
        self.vx_publisher = rospy.Publisher(f'vx_{car_number}', Float32, queue_size=10)
        self.vy_publisher = rospy.Publisher(f'vy_{car_number}', Float32, queue_size=10)

    def callback_safety(self, safety_val_incoming):
        self.safety_value = safety_val_incoming.data

    def callback_steering(self, steer):
        self.steering = steer.data

    def callback_throttle(self, throttle):
        self.throttle = throttle.data if self.safety_value == 1 else 0.0

    def forward_integrate_1_timestep(self):
        t0, t_bound = 0, self.dt_int
        y0 = np.array([self.throttle, self.steering] + self.state)
        RK45_output = integrate.RK45(self.vehicle_model, t0, y0, t_bound)
        while RK45_output.status != 'finished':
            RK45_output.step()
        z_next = RK45_output.y
        self.state = z_next[2:].tolist()

        if self.vehicle_model == kinematic_bicycle:
            steering_angle = steer_angle(z_next[1])
            w = z_next[5] * np.tan(steering_angle) / l
            vy = l_r * w
            self.state[4] = vy
            self.state[5] = w

        theta = self.state[2]
        vx = self.state[3] * np.cos(theta) - self.state[4] * np.sin(theta)
        vy = self.state[3] * np.sin(theta) + self.state[4] * np.cos(theta)

        vicon_msg = PoseWithCovarianceStamped()
        vicon_msg.header.stamp = rospy.Time.now()
        vicon_msg.header.frame_id = 'map'
        vicon_msg.pose.pose.position.x = self.state[0]
        vicon_msg.pose.pose.position.y = self.state[1]
        quaternion = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, theta)
        vicon_msg.pose.pose.orientation.x = quaternion[0]
        vicon_msg.pose.pose.orientation.y = quaternion[1]
        vicon_msg.pose.pose.orientation.z = quaternion[2]
        vicon_msg.pose.pose.orientation.w = quaternion[3]
        self.pub_motion_capture_state.publish(vicon_msg)

        sensor_msg = Float32MultiArray()
        elapsed_time = rospy.Time.now() - self.initial_time
        sensor_msg.data = [
            elapsed_time.to_sec(), 0.0, 0.0, 0.0, 0.0,
            self.state[5], vx, vy, self.safety_value, self.throttle, self.steering
        ]
        self.pub_sens_input.publish(sensor_msg)
        self.vx_publisher.publish(vx)
        self.vy_publisher.publish(vy)

if __name__ == '__main__':
    try:
        rospy.init_node('Vehicles_Integrator_node', anonymous=True)
        dt_int = 0.01
        vehicle_model = kinematic_bicycle
        initial_state_1 = [0, 0, 0, 0.0, 0, 0]
        car_number_1 = 1
        vehicle_1_integrator = Forward_intergrate_vehicle(car_number_1, vehicle_model, initial_state_1, dt_int)

        rate = rospy.Rate(1 / dt_int)
        while not rospy.is_shutdown():
            vehicle_1_integrator.forward_integrate_1_timestep()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
