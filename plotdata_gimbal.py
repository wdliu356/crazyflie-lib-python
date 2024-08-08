"""
Plot data from previous log file
"""
import time
import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

def conv(s):
	try:
		s=float(s)
	except ValueError:
		pass
	return s

def openCSVMatrix(filename = 'filename'):
	rows = []
	with open(filename) as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		for row in reader:
			newrow = []
			for cell in row:
				newcell = conv(cell)
				newrow.append(newcell)
			rows.append(newrow)
	M = np.array(rows)
	return M

def openCSVLog(filename = 'filename'):
	fields = []
	rows = []
	with open(filename) as csvfile:
		reader = csv.reader(csvfile, delimiter=',')
		fields = next(reader)
		for row in reader:
			newrow = []
			for cell in row:
				newcell = conv(cell)
				newrow.append(newcell)
			rows.append(newrow)
	M = np.mat(rows)
	return M

def rpy_YZX(quat0, quat1, quat2, quat3):
	rpy = np.zeros([len(quat0),3])
	for i in range(len(quat0)):
		roll = np.arctan2(-2*(quat2[i]*quat3[i]-quat0[i]*quat1[i]), 1-2*(quat1[i]**2+quat3[i]**2))
		pitch = np.arctan2(-2*(quat1[i]*quat3[i]-quat0[i]*quat2[i]), 1-2*(quat2[i]**2+quat3[i]**2))
		yaw = np.arcsin(2*(quat1[i]*quat2[i]+quat0[i]*quat3[i]))
		rpy[i,:] = [roll,pitch,yaw]
	return rpy

def rpy_ZYX(quat0, quat1, quat2, quat3):
	rpy = np.zeros([len(quat0),3])
	for i in range(len(quat0)):
		roll = np.arctan2(2*(quat0[i]*quat1[i]+quat2[i]*quat3[i]), 1-2*(quat1[i]**2+quat2[i]**2))
		pitch = np.arcsin(-2*(quat1[i]*quat3[i]-quat0[i]*quat2[i]))
		yaw = np.arctan2(2*(quat1[i]*quat2[i]+quat0[i]*quat3[i]), 1-2*(quat2[i]**2+quat3[i]**2))
		rpy[i,:] = [roll,pitch,yaw]
	return rpy


if __name__ == '__main__':

	# z_offset = 1.2
	# fields = []
	# rows = []
	# # u_ILC = openCSVMatrix('./ILC_input/ILC_input.txt')
	# des_traj_file = openCSVMatrix('./ILC/smooth_impulse.txt')
	# des_traj = des_traj_file[:,0] + z_offset
	# N = len(des_traj)

	log_memory_array = openCSVLog('./log_test_0922/log_0923_013450.txt')

	timestamp = log_memory_array[:, 0] - log_memory_array[0, 0]
	i = 2
	pos_x = log_memory_array[:, i]; i+=1
	pos_y = log_memory_array[:, i]; i+=1
	pos_z = log_memory_array[:, i]; i+=1
	vel_x = log_memory_array[:, i]; i+=1
	vel_y = log_memory_array[:, i]; i+=1
	vel_z = log_memory_array[:, i]; i+=1
	roll = log_memory_array[:, i]; i+=1
	pitch = log_memory_array[:, i]; i+=1
	yaw = log_memory_array[:, i]; i+=1
	agv_x = log_memory_array[:, i]; i+=1
	agv_y = log_memory_array[:, i]; i+=1
	agv_z = log_memory_array[:, i]; i+=1
	x_ref = log_memory_array[:, i]; i+=1
	y_ref = log_memory_array[:, i]; i+=1
	z_ref = log_memory_array[:, i]; i+=1
	roll_ref = log_memory_array[:, i]; i+=1
	pitch_ref = log_memory_array[:, i]; i+=1
	yaw_ref = log_memory_array[:, i]; i+=1
	x_vel_ref = log_memory_array[:, i]; i+=1
	y_vel_ref = log_memory_array[:, i]; i+=1
	z_vel_ref = log_memory_array[:, i]; i+=1
	agv_x_ref = log_memory_array[:, i]; i+=1
	agv_y_ref = log_memory_array[:, i]; i+=1
	agv_z_ref = log_memory_array[:, i]; i+=1
	alpha0_ref = log_memory_array[:, i]; i+=1
	alpha1_ref = log_memory_array[:, i]; i+=1
	alpha2_ref = log_memory_array[:, i]; i+=1
	alpha3_ref = log_memory_array[:, i]; i+=1
	beta0_ref = log_memory_array[:, i]; i+=1
	beta1_ref = log_memory_array[:, i]; i+=1
	beta2_ref = log_memory_array[:, i]; i+=1
	beta3_ref = log_memory_array[:, i]; i+=1
	thrust0_ref = log_memory_array[:, i]; i+=1
	thrust1_ref = log_memory_array[:, i]; i+=1
	thrust2_ref = log_memory_array[:, i]; i+=1
	thrust3_ref = log_memory_array[:, i]; i+=1
	quat0 = log_memory_array[:, i]; i+=1
	quat1 = log_memory_array[:, i]; i+=1
	quat2 = log_memory_array[:, i]; i+=1
	quat3 = log_memory_array[:, i]; i+=1
	u1x = log_memory_array[:, i]; i+=1
	u1y = log_memory_array[:, i]; i+=1
	u1z = log_memory_array[:, i]; i+=1
	u2x = log_memory_array[:, i]; i+=1
	u2y = log_memory_array[:, i]; i+=1
	u2z = log_memory_array[:, i]; i+=1

	erx = log_memory_array[:, i]; i+=1
	ery = log_memory_array[:, i]; i+=1
	erz = log_memory_array[:, i]; i+=1
	erix = log_memory_array[:, i]; i+=1
	eriy = log_memory_array[:, i]; i+=1
	eriz = log_memory_array[:, i]; i+=1


	alpha0_meas = log_memory_array[:, i]; i+=1
	alpha1_meas = log_memory_array[:, i]; i+=1
	alpha2_meas = log_memory_array[:, i]; i+=1
	alpha3_meas = log_memory_array[:, i]; i+=1
	beta0_meas = log_memory_array[:, i]; i+=1
	beta1_meas = log_memory_array[:, i]; i+=1
	beta2_meas = log_memory_array[:, i]; i+=1
	beta3_meas = log_memory_array[:, i]; i+=1
	u_alpha0 = log_memory_array[:, i]; i+=1
	u_alpha1 = log_memory_array[:, i]; i+=1
	u_alpha2 = log_memory_array[:, i]; i+=1
	u_alpha3 = log_memory_array[:, i]; i+=1
	u_beta0 = log_memory_array[:, i]; i+=1
	u_beta1 = log_memory_array[:, i]; i+=1
	u_beta2 = log_memory_array[:, i]; i+=1
	u_beta3 = log_memory_array[:, i]; i+=1

	quat = np.array([quat0,quat1,quat2,quat3])

	rpy = rpy_YZX(quat0,quat1,quat2,quat3)

	# define trajectory start condition here
	traj_start = 0
	for j in range(len(x_ref)):
		if x_ref[j] != 0:
			traj_start = j - 1
			break

	# plt.figure(1)
	# plt.subplot(3,2,1)
	# plt.plot(timestamp, pos_z, 'b',timestamp[traj_start:(traj_start + 2*N)][::2], des_traj, 'k--')
	# plt.ylabel('z tracking')
	# plt.grid(True)
	
	# plt.subplot(3,2,1)
	# plt.plot(timestamp,vel_x,'r',timestamp,vel_y,'g',timestamp,vel_z, 'b')
	# plt.ylabel('vel')	# plt.figure(2)
	# plt.subplot(4,1,1)
	# plt.plot(timestamp,alpha0_ref,'k--', timestamp,alpha0_meas,'b')
	# plt.ylabel('alpha 0')
	# plt.grid(True)
	# plt.subplot(4,1,2)
	# plt.plot(timestamp,alpha1_ref,'k--', timestamp,alpha1_meas,'b')
	# plt.ylabel('alpha 1')
	# plt.grid(True)
	# plt.subplot(4,1,3)
	# plt.plot(timestamp,alpha2_ref,'k--', timestamp,alpha2_meas,'b')
	# plt.ylabel('alpha 2')
	# plt.grid(True)
	# plt.subplot(4,1,4)
	# plt.plot(timestamp,alpha3_ref,'k--', timestamp,alpha3_meas,'b')
	# plt.ylabel('alpha 3')
	# plt.grid(True)

	# plt.figure(3)
	# plt.subplot(4,1,1)
	# plt.plot(timestamp,beta0_ref,'k--', timestamp,beta0_meas,'b')
	# plt.ylabel('beta 0')
	# plt.grid(True)
	# plt.subplot(4,1,2)
	# plt.plot(timestamp,beta1_ref,'k--', timestamp,beta1_meas,'b')
	# plt.ylabel('beta 1')
	# plt.grid(True)
	# plt.subplot(4,1,3)
	# plt.plot(timestamp,beta2_ref,'k--', timestamp,beta2_meas,'b')
	# plt.ylabel('beta 2')
	# plt.grid(True)
	# plt.subplot(4,1,4)
	# plt.plot(timestamp,beta3_ref,'k--', timestamp,beta3_meas,'b')
	# plt.ylabel('beta 3')
	# plt.grid(True)
	# plt.grid(True)
	# plt.subplot(3,2,2)
	# plt.plot(timestamp,agv_x,'r',timestamp,agv_y,'g',timestamp,agv_z, 'b')
	# plt.ylabel('agv')
	# plt.grid(True)
	plt.subplot(4,2,1)
	plt.plot(timestamp,roll,'r',timestamp,pitch,'g',timestamp,yaw, 'b',timestamp,roll_ref,'r--',timestamp,pitch_ref,'g--',timestamp,yaw_ref, 'b--')
	plt.ylabel('rpy from ctrl')
	plt.grid(True)
	plt.subplot(4,2,6)
	plt.plot(timestamp,agv_x,'r',timestamp,agv_y,'g',timestamp,agv_z, 'b',timestamp,agv_x_ref,'r--',timestamp,agv_y_ref,'g--',timestamp,agv_z_ref, 'b--')
	plt.ylabel('linear v')
	plt.grid(True)
	plt.subplot(4,2,2)
	plt.plot(timestamp,pos_x,'r',timestamp,pos_y,'g',timestamp,pos_z, 'b',timestamp,x_ref,'r--',timestamp,y_ref,'g--',timestamp,z_ref, 'b--')
	plt.ylabel('pos')
	plt.grid(True)
	plt.subplot(4,2,3)
	plt.plot(timestamp,thrust0_ref,'r',timestamp,thrust1_ref,'g',timestamp,thrust2_ref,'b',timestamp,thrust3_ref,'m')
	plt.ylabel('thrust')
	plt.grid(True)
	plt.subplot(4,2,4)
	#plt.plot(timestamp,alpha2_ref,'b--',timestamp,alpha3_ref,'m--')
	#plt.plot(timestamp, alpha2_meas, 'b', timestamp, alpha3_meas,'m')
	plt.plot(timestamp,alpha0_ref,'r--',timestamp,alpha1_ref,'g--',timestamp,alpha2_ref,'b--',timestamp,alpha3_ref,'m--')
	plt.plot(timestamp, alpha0_meas, 'r',timestamp, alpha1_meas, 'g', timestamp, alpha2_meas, 'b', timestamp, alpha3_meas,'m')
	plt.ylabel('alpha')
	plt.grid(True)
	plt.subplot(4,2,5)
	plt.plot(timestamp,beta0_ref,'r--',timestamp,beta1_ref,'g--',timestamp,beta2_ref,'b--',timestamp,beta3_ref,'m--')
	plt.plot(timestamp,beta0_meas,'r',timestamp,beta1_meas,'g',timestamp,beta2_meas,'b',timestamp,beta3_meas,'m')
	plt.ylabel('beta')
	plt.grid(True)
	# plt.subplot(3,2,4)
	# plt.plot(timestamp,rpy[:,0],'r',timestamp,rpy[:,1],'g',timestamp,rpy[:,2], 'b',timestamp,pitch_ref,'k--')
	# plt.ylabel('rpy')
	# plt.grid(True)
	# plt.subplot(3,2,5)
	# plt.plot(u_ILC)
	# plt.ylabel('u_ILC')
	# plt.grid(True)
	plt.subplot(4,2,7)
	plt.plot(timestamp,u1x,'r',timestamp,u1y,'g',timestamp,u1z,'b')
	plt.ylabel('u1')
	plt.grid(True)
	plt.subplot(4,2,8)
	plt.plot(timestamp,u2x,'r',timestamp,u2y,'g',timestamp,u2z,'b')
	plt.ylabel('u2')
	plt.grid(True)
	
	plt.figure(2)
	plt.subplot(3,1,1)
	plt.plot(timestamp,erx,'r',timestamp,ery,'g',timestamp,erz,'b')
	plt.ylabel('err_rpy')
	plt.grid(True)
	plt.subplot(3,1,2)
	plt.plot(timestamp,erix,'r',timestamp,eriy,'g',timestamp,eriz,'b')
	plt.ylabel('erri_rpy')
	plt.grid(True)
	plt.subplot(3, 1, 3)
	plt.plot(timestamp,vel_x,'r',timestamp,vel_y,'g',timestamp,vel_z, 'b',timestamp,x_vel_ref,'r--',timestamp,y_vel_ref,'g--',timestamp,z_vel_ref, 'b--')
	# plt.subplot(4,1,1)
	# plt.plot(timestamp,alpha0_ref,'k--', timestamp,alpha0_meas,'b')
	# plt.ylabel('alpha 0')
	# plt.grid(True)
	# plt.subplot(4,1,2)
	# plt.plot(timestamp,alpha1_ref,'k--', timestamp,alpha1_meas,'b')
	# plt.ylabel('alpha 1')
	# plt.grid(True)
	# plt.subplot(4,1,3)
	# plt.plot(timestamp,alpha2_ref,'k--', timestamp,alpha2_meas,'b')
	# plt.ylabel('alpha 2')
	# plt.grid(True)
	# plt.subplot(4,1,4)
	# plt.plot(timestamp,alpha3_ref,'k--', timestamp,alpha3_meas,'b')
	# plt.ylabel('alpha 3')
	# plt.grid(True)

	# plt.figure(3)
	# plt.subplot(4,1,1)
	# plt.plot(timestamp,beta0_ref,'k--', timestamp,beta0_meas,'b')
	# plt.ylabel('beta 0')
	# plt.grid(True)
	# plt.subplot(4,1,2)
	# plt.plot(timestamp,beta1_ref,'k--', timestamp,beta1_meas,'b')
	# plt.ylabel('beta 1')
	# plt.grid(True)
	# plt.subplot(4,1,3)
	# plt.plot(timestamp,beta2_ref,'k--', timestamp,beta2_meas,'b')
	# plt.ylabel('beta 2')
	# plt.grid(True)
	# plt.subplot(4,1,4)
	# plt.plot(timestamp,beta3_ref,'k--', timestamp,beta3_meas,'b')
	# plt.ylabel('beta 3')
	# plt.grid(True)

	
	plt.show()