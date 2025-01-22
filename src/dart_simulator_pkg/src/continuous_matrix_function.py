import numpy as np
from funct_fin import steer_angle, rolling_friction, motor_force, F_friction_due_to_steering, slip_angles, lateral_tire_forces, friction

def continuous_matrices(index, steering_input, vx, vy, w, tau):
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
    delta0 = steer_angle(steering_input[index - 1])
    alpha_f0, alpha_r0 = slip_angles(vx[index - 1], vy[index - 1], w[index - 1], delta0)
    F_y_f_0, F_y_r_0 = lateral_tire_forces(alpha_f0, alpha_r0)

    # LONGITUDINAL DYNAMICS
    F_rolling_0 = rolling_friction(vx[index - 1])
    F_m_0 = motor_force(tau[index - 1], vx[index - 1])
    F_fric_0 = F_friction_due_to_steering(delta0, vx[index - 1])
    F_x_0 = np.sum(F_rolling_0) + np.sum(F_m_0) + np.sum(F_fric_0)
    F_x_0_f = Cf * F_x_0
    F_x_0_r = Cr * F_x_0

    # SET UP MATRICES
    F_0 = np.array([
        1/m * (F_x_0_r + F_x_0_f * np.cos(delta0)) + w[index - 1] * vy[index - 1],
        1/m * (F_x_0_f * np.sin(delta0)) - w[index - 1] * vx[index - 1],
        lf/Jz * (F_x_0_f * np.sin(delta0))
    ]).reshape(-1, 1)

    G_0 = np.array([
        -1/m * F_y_f_0 * np.sin(delta0),
        1/m * (F_y_r_0 + F_y_f_0 * np.cos(delta0)),
        1/Jz * (lf * F_y_f_0 * np.cos(delta0) - lr * F_y_r_0)
    ]).reshape(-1, 1)

    return F_0, G_0