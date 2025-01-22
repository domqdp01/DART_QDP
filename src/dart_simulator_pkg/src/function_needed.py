import numpy as np

# Function to transform steering input into steering angle
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


# Function to evaluate slip angles for front and rear wheels
def evaluate_slip_angles(vx, vy, w, lf, lr, steer_angle):
    vy_wheel_f = vy + lf * w  # Lateral velocity of the front wheel
    vy_wheel_r = vy - lr * w  # Lateral velocity of the rear wheel
    vx_wheel_f = np.cos(-steer_angle) * vx - np.sin(-steer_angle) * (vy + lf * w)

    # Velocity correction terms
    Vx_correction_term_f = 1 * np.exp(-3 * vx_wheel_f**2)
    Vx_correction_term_r = 1 * np.exp(-3 * vx**2)

    Vx_f = vx_wheel_f + Vx_correction_term_f
    Vx_r = vx + Vx_correction_term_r

    # Compute slip angles
    alpha_f = np.arctan2(vy_wheel_f, Vx_f)
    alpha_r = np.arctan2(vy_wheel_r, Vx_r)
    
    return alpha_f, alpha_r

# Function to evaluate lateral tire force
def lateral_tire_force(alpha, d_t, c_t, b_t, m_wheel):
    F_y = m_wheel * 9.81 * d_t * np.sin(c_t * np.arctan(b_t * alpha))
    return F_y 

# Function to calculate rolling friction
def rolling_friction(vx, a_f, b_f, c_f, d_f):
    F_rolling = - (a_f * np.tanh(b_f * vx) + c_f * vx + d_f * vx**2)
    return F_rolling

# Function to calculate motor force
def motor_force(throttle_filtered, v, a_m, b_m, c_m):
    w_m = 0.5 * (np.tanh(100 * (throttle_filtered + c_m)) + 1)  # Weighting function
    Fx = (a_m - b_m * v) * w_m * (throttle_filtered + c_m)
    return Fx

# Function to compute additional friction due to steering
def F_friction_due_to_steering(steer_angle, vx, a, b, d, e):
    friction_term = a + (b * steer_angle * np.tanh(30 * steer_angle))
    vx_term = - (0.5 + 0.5 * np.tanh(20 * (vx - 0.3))) * (e + d * (vx - 0.5))
    F_frict = friction_term * vx_term
    return F_frict

# Function to define model parameters
def model_parameters():
    return [25.3585, 4.8153, -0.1638, 0.0843,  # Motor parameters
            1.266, 7.666, 0.7393, -0.1123,    # Friction parameters
            1.3929, 0.3658, -0.0270, 0.5148, 1.0230,  # Steering parameters
            -0.8407, 0.8407, 8.5980, -0.8547, 0.9591, 11.5493,  # Tire forces
            -0.1183, 5.9159, 0.2262, 0.7793,  # Friction due to steering
            0.1285, 0.1406, 2.7244]  # Dynamics parameters

# Class to manage model functions
class model_functions():
    
    [a_m_self, b_m_self, c_m_self, d_m_self,
    a_f_self, b_f_self, c_f_self, d_f_self,
    a_s_self, b_s_self, c_s_self, d_s_self, e_s_self,
    d_t_f_self, c_t_f_self, b_t_f_self,d_t_r_self, c_t_r_self, b_t_r_self,
    a_stfr_self, b_stfr_self,d_stfr_self,e_stfr_self,
    k_stdn_self,k_pitch_self,w_natural_Hz_pitch_self] = model_parameters()

    def __init__(self):
        # this is just a class to collect all the functions that are used to model the dynamics
        pass

    def minmax_scale_hm(self,min,max,normalized_value):
        # normalized value should be between 0 and 1
        return min + normalized_value * (max-min)

    def steering_2_steering_angle(self,steering_command,a_s,b_s,c_s,d_s,e_s):
        w_s = 0.5 * (np.tanh(30*(steering_command+c_s))+1)
        steering_angle1 = b_s * np.tanh(a_s * (steering_command + c_s))
        steering_angle2 = d_s * np.tanh(e_s * (steering_command + c_s))
        steering_angle = (w_s)*steering_angle1+(1-w_s)*steering_angle2
        return steering_angle
    
    def rolling_friction(self,vx,a_f,b_f,c_f,d_f):
       
        F_rolling = - ( a_f * np.tanh(b_f  * vx) + c_f * vx + d_f * vx**2 )
        return F_rolling
    

    def motor_force(self,throttle_filtered,v,a_m,b_m,c_m):

        w_m = 0.5 * (np.tanh(100*(throttle_filtered+c_m))+1)
        Fx =  (a_m - b_m * v) * w_m * (throttle_filtered+c_m)
        return Fx
    
    def evaluate_slip_angles(self,vx,vy,w,lf,lr,steer_angle):
        vy_wheel_f,vy_wheel_r = self.evalaute_wheel_lateral_velocities(vx,vy,w,steer_angle,lf,lr)


        # do the same but for numpy
        vx_wheel_f = np.cos(-steer_angle) * vx - np.sin(-steer_angle)*(vy + lf*w)

        Vx_correction_term_f = 1 * np.exp(-3*vx_wheel_f**2) 
        Vx_correction_term_r = 1 *  np.exp(-3*vx**2) 

        Vx_f = vx_wheel_f + Vx_correction_term_f
        Vx_r = vx + Vx_correction_term_r
        
        alpha_f = np.arctan2(vy_wheel_f,Vx_f)
        alpha_r = np.arctan2(vy_wheel_r,Vx_r)
            
        return alpha_f,alpha_r
    
    def lateral_forces_activation_term(self,vx):
            return np.tanh(100 * vx**2)

    def lateral_tire_force(self,alpha,d_t,c_t,b_t,m_wheel):
        
        F_y = m_wheel * 9.81 * d_t * np.sin(c_t * np.arctan(b_t * alpha))
        return F_y 
    

    def evalaute_wheel_lateral_velocities(self,vx,vy,w,steer_angle,lf,lr):

        Vy_wheel_f = - np.sin(steer_angle) * vx + np.cos(steer_angle)*(vy + lf*w) 
        Vy_wheel_r = vy - lr*w
        return Vy_wheel_f,Vy_wheel_r
    

    def F_friction_due_to_steering(self,steer_angle,vx,a,b,d,e):        # evaluate forward force
        friction_term = a + (b * steer_angle * np.tanh(30 * steer_angle))
        vx_term =  - (0.5+0.5 *np.tanh(20*(vx-0.3))) * (e + d * (vx-0.5))

        return  vx_term * friction_term


    
    def solve_rigid_body_dynamics(self,vx,vy,w,steer_angle,Fx_front,Fx_rear,Fy_wheel_f,Fy_wheel_r,lf,lr,m,Jz):

        # evaluate centripetal acceleration
        a_cent_x = + w * vy
        a_cent_y = - w * vx

        # evaluate body forces
        Fx_body =  Fx_front*(np.cos(steer_angle))+ Fx_rear + Fy_wheel_f * (-np.sin(steer_angle))

        Fy_body =  Fx_front*(np.sin(steer_angle)) + Fy_wheel_f * (np.cos(steer_angle)) + Fy_wheel_r

        M       = Fx_front * (+np.sin(steer_angle)*lf) + Fy_wheel_f * (np.cos(steer_angle)*lf)+\
                Fy_wheel_r * (-lr)
        
        acc_x = Fx_body/m + a_cent_x
        acc_y = Fy_body/m + a_cent_y
        acc_w = M/Jz
        
        return acc_x,acc_y,acc_w
    


    
    def critically_damped_2nd_order_dynamics_numpy(self,x_dot,x,forcing_term,w_Hz):
        z = 1 # critically damped system
        w_natural = w_Hz * 2 * np.pi # convert to rad/s
        x_dot_dot = w_natural ** 2 * (forcing_term - x) - 2* w_natural * z * x_dot
        return x_dot_dot


    def produce_past_action_coefficients_1st_oder(self,C,length,dt):

        
        k_vec = np.zeros((length,1))
        for i in range(length):
            k_vec[i] = self.impulse_response_1st_oder(i*dt,C) 
        k_vec = k_vec * dt # the dt is really important to get the amplitude right
        return k_vec 


    def impulse_response_1st_oder(self,t,C):
        return np.exp(-t/C)*1/C



    def produce_past_action_coefficients_1st_oder_step_response(self,C,length,dt):

            k_vec = np.zeros((length,1))
            for i in range(1,length): # the first value is zero because it has not had time to act yet
                k_vec[i] = self.step_response_1st_oder(i*dt,C) - self.step_response_1st_oder((i-1)*dt,C)  
            
            return k_vec 
    

    def step_response_1st_oder(self,t,C):
        return 1 - np.exp(-t/C)
        
    def continuous_time_1st_order_dynamics(self,x,forcing_term,C):
        x_dot = 1/C * (forcing_term - x)
        return x_dot

# Function to process throttle dynamics data
def throttle_dynamics_data_processing(df_raw_data):
    mf = model_functions()  # Instantiate model functions class

    dt = df_raw_data['vicon time'].diff().mean()  # Calculate average time step
    th = 0  # Initialize throttle dynamics
    filtered_throttle = np.zeros(df_raw_data.shape[0])

    # Refine ground truth with higher resolution for numerical accuracy
    ground_truth_refinement = 100
    for t in range(1, len(filtered_throttle)):
        for k in range(ground_truth_refinement):
            th_dot = mf.continuous_time_1st_order_dynamics(
                th, df_raw_data['throttle'].iloc[t-1], mf.d_m_self
            )
            th += dt / ground_truth_refinement * th_dot
        filtered_throttle[t] = th

    return filtered_throttle
