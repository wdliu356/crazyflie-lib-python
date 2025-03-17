import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('log_test_0306_new_model/log_0306_142153.txt', delimiter=',', skiprows=1)
timestamp = data[:,0]
dt = data[:,1]
pos = data[:,2:5]
pos_ref = data[:,5:8]
vel = data[:,8:11]
vel_ref = data[:,11:14]
rpy = data[:,14:17]
rpy_ref = data[:,17:20]
rpyPID = data[:,20:23]
agv = data[:,23:26]
torque = data[:,26:29]
w = data[:,29:33]

plt.figure()
plt.plot(timestamp, rpy[:,0], label='roll',color='r')
plt.plot(timestamp, rpy_ref[:,0], label='roll ref', linestyle='--',color='r')
plt.plot(timestamp, rpy[:,1], label='pitch',color='g')
plt.plot(timestamp, rpy_ref[:,1], label='pitch ref', linestyle='--',color='g')
plt.plot(timestamp, rpy[:,2], label='yaw',color='b')
plt.plot(timestamp, rpy_ref[:,2], label='yaw ref', linestyle='--',color='b')
plt.xlabel('Time')  # x-axis label
plt.ylabel('Angle')  # y-axis label
plt.title('Angle vs Time')  # title of the plot
plt.grid()
plt.legend()
plt.figure()
plt.plot(timestamp,torque[:,0], label='x torque',color='r')
plt.plot(timestamp,torque[:,1], label='y torque',color='g')
plt.plot(timestamp,torque[:,2], label='z torque',color='b')
plt.xlabel('Time')  # x-axis label
plt.ylabel('Torque')  # y-axis label
plt.title('Torque vs Time')  # title of the plot
plt.grid()
plt.legend()
plt.show()
