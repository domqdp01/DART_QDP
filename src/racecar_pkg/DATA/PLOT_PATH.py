import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec

# File path for the dataset
# file_path = 'car_1_Datarecording_12_06_2024_09_48_04.csv'
# file_path = 'circle.csv'
# file_path = 'circle_fin.csv'
# file_path = 'car_1_Datarecording_12_11_2024_14_22_11.csv'
# file_path = 'car_1_Datarecording_12_11_2024_14_59_16.csv'
# file_path = 'car_1_Datarecording_12_16_2024_14_41_11.csv'
# file_path = 'car_1_Datarecording_12_16_2024_15_08_13.csv'
# file_path = 'car_1_Datarecording_12_16_2024_15_17_32.csv'
file_path = 'car_1_Datarecording_12_16_2024_15_42_03.csv'

# Load the dataset
df = pd.read_csv(file_path)

# Extract relevant data columns

vy = df['velocity y'].values           # Velocity in the y direction
x = df['vicon x'].values               # Position in x
y = df['vicon y'].values               # Position in y
vx = df['vel encoder'].values          # Velocity from encoder
t = df['elapsed time sensors'].values  # Elapsed time from sensors
yaw = df['vicon yaw'].values           # Yaw angle from vicon
w = df['W (IMU)'].values               # Angular velocity (W) from IMU

# Create the figure and define a custom layout
gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])
fig = plt.figure(figsize=(10, 8))

# First subplot: Trajectory plot (occupies the entire first row)
ax1 = fig.add_subplot(gs[0, :])  # Spans all columns in the first row
ax1.plot(x, y, label='Trajectory', zorder=1)
ax1.scatter(x[0], y[0], color='red', s=30, label='Starting point', zorder=2)
ax1.scatter(x[-1], y[-1], color='black', s=30, label='Ending point', zorder=2)
ax1.set_title("CAR SIMULATOR TRAJECTORY", fontsize=14)
ax1.set_xlabel("X", fontsize=12)
ax1.set_ylabel("Y", fontsize=12)
ax1.legend(loc='upper left')

# Second subplot: Yaw angle vs. time
top_ax2 = fig.add_subplot(gs[1, 0])
top_ax2.plot(t, yaw, label='Yaw')
top_ax2.set_xlabel("t", fontsize=12)
top_ax2.set_ylabel("YAW", fontsize=12)
top_ax2.legend(loc='best')

# Third subplot: Angular velocity (W) vs. time
top_ax3 = fig.add_subplot(gs[1, 1])
top_ax3.plot(t, w, label='W')
top_ax3.set_xlabel("t", fontsize=12)
top_ax3.set_ylabel("W", fontsize=12)
top_ax3.legend(loc='best')

# Fourth subplot: Velocity in the x direction vs. time
bottom_ax4 = fig.add_subplot(gs[2, 0])
bottom_ax4.plot(t, vx, label='Vx')
bottom_ax4.set_xlabel("t", fontsize=12)
bottom_ax4.set_ylabel("vx", fontsize=12)
bottom_ax4.legend(loc='best')

# Fifth subplot: Velocity in the y direction vs. time
bottom_ax5 = fig.add_subplot(gs[2, 1])
bottom_ax5.plot(t, vy, label='Vy')
bottom_ax5.set_xlabel("t", fontsize=12)
bottom_ax5.set_ylabel("vy", fontsize=12)
bottom_ax5.legend(loc='best')

# Optimize layout to prevent overlap
plt.tight_layout()
plt.show()
