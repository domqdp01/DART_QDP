import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.gridspec as gridspec


# file_path = 'car_1_Datarecording_12_03_2024_11_43_26.csv'
# file_path = 'car_1_Datarecording_12_03_2024_11_46_00.csv'
# file_path = 'car_1_Datarecording_12_03_2024_15_45_25.csv'
# file_path = 'car_1_Datarecording_12_03_2024_15_54_54.csv'
# file_path = 'car_1_Datarecording_12_03_2024_16_32_22.csv'
# file_path = 'car_1_Datarecording_12_03_2024_16_34_36.csv'
# file_path = 'car_1_Datarecording_12_03_2024_16_47_41.csv'
# file_path = 'car_1_Datarecording_12_03_2024_16_53_50.csv'
# file_path = 'car_1_Datarecording_12_04_2024_11_34_51.csv'
# file_path = 'car_1_Datarecording_12_04_2024_12_20_56.csv'
# file_path = 'car_1_Datarecording_12_04_2024_12_34_16.csv'
# file_path = 'car_1_Datarecording_12_04_2024_13_17_47.csv'
# file_path = 'car_1_Datarecording_12_04_2024_13_19_56.csv'
# file_path = 'car_1_Datarecording_12_04_2024_13_21_43.csv'
# file_path = 'car_1_Datarecording_12_04_2024_13_23_19.csv'
file_path = 'car_1_Datarecording_12_04_2024_13_38_47.csv'

df = pd.read_csv(file_path)
# print(df.columns)
vx = df['velocity x'].values
vy = df['velocity y'].values
x = df['vicon x'].values
y = df['vicon y'].values
v = df['vel encoder'].values
t = df['elapsed time sensors'].values
yaw = df['vicon yaw'].values
w = df['W (IMU)'].values

# Creazione della figura e layout personalizzato
gs = gridspec.GridSpec(3, 2, height_ratios=[2, 1, 1])
fig = plt.figure(figsize=(10, 8))

# Primo subplot: occupa una riga intera
ax1 = fig.add_subplot(gs[0, :])  # Occupa tutte le colonne della prima riga
ax1.plot(x, y, label='Trajectory', zorder=1)
ax1.scatter(x[0], y[0], color='red', s=30, label='Starting point', zorder=2)
ax1.scatter(x[-1], y[-1], color='black', s=30, label='Ending point', zorder=2)
ax1.set_title("CAR SIMULATOR TRAJECTORY", fontsize=14)
ax1.set_xlabel("X", fontsize=12)
ax1.set_ylabel("Y", fontsize=12)
ax1.legend(loc='best')

# Secondo subplot: Yaw
top_ax2 = fig.add_subplot(gs[1, 0])
top_ax2.plot(t, yaw, label='Yaw')
top_ax2.set_xlabel("t", fontsize=12)
top_ax2.set_ylabel("YAW", fontsize=12)
top_ax2.legend(loc='best')

# Terzo subplot: W
top_ax3 = fig.add_subplot(gs[1, 1])
top_ax3.plot(t, w, label='W')
top_ax3.set_xlabel("t", fontsize=12)
top_ax3.set_ylabel("W", fontsize=12)
top_ax3.legend(loc='best')

# Quarto subplot: Vx
bottom_ax4 = fig.add_subplot(gs[2, 0])
bottom_ax4.plot(t, vx, label='Vx')
bottom_ax4.set_xlabel("t", fontsize=12)
bottom_ax4.set_ylabel("vx", fontsize=12)
bottom_ax4.legend(loc='best')

# Quinto subplot: Vy
bottom_ax5 = fig.add_subplot(gs[2, 1])
bottom_ax5.plot(t, vy, label='Vy')
bottom_ax5.set_xlabel("t", fontsize=12)
bottom_ax5.set_ylabel("vy", fontsize=12)
bottom_ax5.legend(loc='best')

# Ottimizzazione del layout
plt.tight_layout()
plt.show()