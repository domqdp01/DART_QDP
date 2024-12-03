import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# file_path = 'car_1_Datarecording_12_03_2024_11_43_26.csv'
# file_path = 'car_1_Datarecording_12_03_2024_11_46_00.csv'
file_path = 'car_1_Datarecording_12_03_2024_15_45_25.csv'
# file_path = '.csv'
# file_path = '.csv'
# file_path = '.csv'
# file_path = '.csv'
# file_path = '.csv'


df = pd.read_csv(file_path)

x = df['vicon x'].values
y = df['vicon y'].values
v = df['vel encoder'].values
t = df['elapsed time sensors'].values

plt.subplot(2,1,1)
plt.plot(x, y,  label='Trajectory',  zorder=1)
plt.scatter(x[0], y[0], color='red', s=30, label='Starting point', zorder=2)  
plt.scatter(x[-1], y[-1], color='black', s=30, label='Ending point',  zorder=2)  
plt.title("CAR SIMULATOR TRAJECTORY", fontsize=14)
plt.xlabel("X", fontsize=12)
plt.ylabel("Y", fontsize=12)
plt.legend(loc='best')
plt.subplot(2,1,2)
plt.plot(t, v,  label='velocity') 
plt.xlabel("t", fontsize=12)
plt.ylabel("V", fontsize=12)
plt.legend(loc='best')
plt.show()