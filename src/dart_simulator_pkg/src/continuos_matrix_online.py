import numpy as np
from funct_fin import steer_angle, rolling_friction, motor_force, F_friction_due_to_steering, slip_angles, lateral_tire_forces, friction

def continuous_matrices(z, u):
    """
    Computes the continuous-time matrices for lateral and longitudinal dynamics.

    Parameters:
    steering_input (array): Steering angle inputs.
    index (int): Current time step index.
    vx (array): Longitudinal velocity at each time step.
    vy (array): Lateral velocity at each time step.
    w (array): Angular velocity at each time step.
    tau (array): Motor torque at each time step.

    Returns:
    tuple: F_0 and G_0 matrices as numpy arrays.
    """
    l = 0.175
    lr = 0.54 * l
    lf = l - lr
    m = 1.67
    m_front_wheel = 0.847
    m_rear_wheel = 0.733
    Cf = m_front_wheel / m
    Cr = m_rear_wheel / m
    Jz = 0.006513

    # LATERAL DYNAMICS
    alpha_f0, alpha_r0 = slip_angles(z[0], z[1], z[2], u[1])
    F_y_f_0, F_y_r_0 = lateral_tire_forces(alpha_f0, alpha_r0)

    # LONGITUDINAL DYNAMICS
    F_rolling_0 = rolling_friction(z[0])
    F_m_0 = motor_force(u[1], z[0])
    F_x_0 = F_rolling_0 + F_m_0
    F_x_0_f = F_x_0 / 2
    F_x_0_r = F_x_0 / 2


    # SET UP MATRICES
    F_0 = np.array([
        1/m * (F_x_0_r + F_x_0_f * np.cos(u[1])) + z[2] * z[1],
        1/m * (F_x_0_f * np.sin(u[1])) - z[2] * z[0],
        lf/Jz * (F_x_0_f * np.sin(u[1]))
    ]).reshape(-1, 1)

    G_0 = np.array([
        -1/m * F_y_f_0 * np.sin(u[1]),
        1/m * (F_y_r_0 + F_y_f_0 * np.cos(u[1])),
        1/Jz * (lf * F_y_f_0 * np.cos(u[1]) - lr * F_y_r_0)
    ]).reshape(-1, 1)

    return F_0, G_0