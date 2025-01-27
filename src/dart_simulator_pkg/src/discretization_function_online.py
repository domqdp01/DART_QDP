import numpy as np

def compute_discrete_function_terms_single_step_euler(
    previous_state_measurement, 
    # previous_control_input, 
    autonomous_function, 
    input_function
):
    """
    Single step Euler integration for input map g(u) and autonomous map f(x).
    Computes: x_{k} = x_k-1 + dt*f(x_k-1) + dt*g(u_k-1)
    
    Args:
        previous_state_measurement (np.ndarray): Previous state (x_k-1).
        autonomous_function: Function f(x) for the autonomous map.
        input_function: Function g(u) for the input map.

    """
    controller_timestep = 0.2
    integration_timestep = 0.01

    # Initialize variables
    x_discrete = np.zeros_like(previous_state_measurement)
    f_discrete = np.zeros_like(previous_state_measurement)
    g_discrete = np.zeros_like(previous_state_measurement)

    # Compute f_discrete
    f_discrete = previous_state_measurement + integration_timestep * autonomous_function
    
    # Compute g_discrete
    g_discrete = integration_timestep * input_function

    # Compute x_discrete
    x_discrete = f_discrete + g_discrete

    n_state = 3                       # number of states
    I_n = np.eye(n_state, dtype=int)  # dim (3,3)
    H = np.vstack([I_n, -I_n])        # dim (6,3)

    d_up = 0.01                 # noise upper bound
    d_low = - d_up                    # noise lower bound
    h_d = np.concatenate([
        np.full((n_state, 1), d_up),
        np.full((n_state, 1), -d_low)
        ])
    
    A_i = - H @ g_discrete
    b_i = h_d - H @ x_discrete + H @ f_discrete
    

    return A_i, b_i