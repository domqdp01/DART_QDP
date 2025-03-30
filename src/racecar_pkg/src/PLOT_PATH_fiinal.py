import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# file_path = '/home/domenico/DART_QDP/src/racecar_pkg/DATA/car_1_Datarecording_02_18_2025_17_01_28.csv' #clockwise_simulator
# file_path = '/home/domenico/DART_QDP/src/racecar_pkg/DATA/car_1_Datarecording_02_18_2025_17_29_30.csv' #counter_clockwise_simulator
file_path = '/home/domenico/DART_QDP/src/racecar_pkg/DATA/car_1_Datarecording_02_18_2025_17_40_49.csv' # Staight line
df = pd.read_csv(file_path)

# Load data
vy = df['velocity y'].values
x = df['vicon x'].values
y = df['vicon y'].values
vx = df['vel encoder'].values
t = df['elapsed time sensors'].values
yaw = df['vicon yaw'].values
w = df['W (IMU)'].values
tau = df['throttle'].values
steering_input = df['steering'].values

ending_point = len(df)//2

# Values to Set the arrow

h_width = 0.1
h_length = 0.1

#  Create the plot
plt.figure(figsize=(8, 6))
plt.plot(x[:ending_point], y[:ending_point], label='Trajectory', color='b')
plt.scatter([x[0]], [y[0]], color=['g'], label='Starting Point', zorder=3)
plt.scatter([x[ending_point]], [y[ending_point]], color=['r'], label='Ending Point', zorder=3)
mid_index = int(ending_point // 3.8) # Ensure mid_index is an integer
plt.arrow(x[mid_index - 2], y[mid_index - 2], 
          x[mid_index - 1] - x[mid_index - 2], 
          y[mid_index - 1] - y[mid_index - 2], 
          head_width=h_width, head_length=h_length, fc='b', ec='b')

mid_index_2 = int(ending_point // 1.7)  # Ensure mid_index is an integer

plt.arrow(x[mid_index_2 - 2], y[mid_index_2 - 2], 
          x[mid_index_2 - 1] - x[mid_index_2 - 2], 
          y[mid_index_2 - 1] - y[mid_index_2 - 2], 
          head_width=h_width, head_length=h_length, fc='b', ec='b')

mid_index_3 = int(ending_point // 1.3)  # Ensure mid_index is an integer

plt.arrow(x[mid_index_3 - 2], y[mid_index_3 - 2], 
          x[mid_index_3 - 1] - x[mid_index_3 - 2], 
          y[mid_index_3 - 1] - y[mid_index_3 - 2], 
          head_width=h_width, head_length=h_length, fc='b', ec='b')

# Values for setting the axis limits
x_low = 0
x_up = 6
y_low = -5
y_up = 0.5
plt.xlim(x_low, x_up)
plt.ylim(y_low, y_up)
plt.xlabel('Position X [m]')
plt.ylabel('Position Y [m]')
plt.title('Vehicle Trajectory')
plt.legend()
plt.grid()
plt.show()

