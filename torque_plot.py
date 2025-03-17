import numpy as np
import matplotlib.pyplot as plt

## load data from torque_test folder
## The files are in txt file and the data is in the following format:
## timestamp,dt,roll,w0,w1,w2,w3,thrust_ref,roll_torque
##
## The data is loaded into a numpy array
## The data is then plotted using matplotlib
## Plot the relationship between roll and roll_torque and calculate the correlation coefficient
## Plot the calculated roll_torque based on the coefficient

data = np.loadtxt('torque_test/log_0304_140422.txt', delimiter=',', skiprows=1)
roll = data[:,2]
roll_torque = data[:,10]
print('Roll:', roll)
print('Roll Torque:', roll_torque)


## Calculate the correlation coefficient
correlation = np.corrcoef(roll, roll_torque)
# print('Correlation coefficient:', correlation[0,1])

## Calculate the roll_torque based on the correlation coefficient
## The equation is roll_torque = a*roll + b

a = correlation[0,1] * np.std(roll_torque) / np.std(roll)
b = np.mean(roll_torque) - a * np.mean(roll)

calculated_roll_torque = a * roll + b
print('a:', a)
print('b:', b)

plt.plot(roll, roll_torque, 'o', label='Measured Roll Torque')
plt.plot(roll, calculated_roll_torque, label='Calculated Roll Torque')
plt.xlabel('Roll (rad)')
plt.ylabel('Roll Torque')
plt.title('Roll vs Roll Torque')
plt.grid()
plt.legend()
plt.show()


