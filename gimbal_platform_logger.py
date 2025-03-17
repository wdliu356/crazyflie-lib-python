"""
	Logger class. record data and export to a csv file
	and plot
"""
import time
import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

class CombinedLogger:
	
	def __init__(self, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "dt",
							"pos_x", "pos_y", "pos_z",
							"vel_x", "vel_y", "vel_z",
							"roll", "pitch", "yaw",
							"agv_x", "agv_y", "agv_z",
							"x_ref", "y_ref", "z_ref",
							"roll_ref", "pitch_ref", "yaw_ref",
							"x_vel_ref", "y_vel_ref", "z_vel_ref",
							"x_torque", "y_torque", "z_torque",
							"w0","w1","w2","w3",
							"u0","u1","u2","u3",
							"m0req", "m1req", "m2req", "m3req",
							"thrust0", "thrust1", "thrust2", "thrust3",
							"thrust_ref",
							"roll_vicon", "pitch_vicon", "yaw_vicon",
							"acc_x", "acc_y", "acc_z",
							"roll_received", "pitch_received", "yaw_received",]]
							# "roll_torque"]]
							 # error rotation integration

	def log_append(self, timestamp, dt, 
						 pos, vel, rpy, agv, 
						 pos_ref, rpy_ref, vel_ref,torque,
						 w,u,mreq,thrust,
						#  u,w,qfb,
						 thrust_ref,rpy_vicon,acc,att_received):#,roll_torque):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		d = round(d,3)
		"""
		self.log_memory.append([timestamp, dt,
								pos[0], pos[1], pos[2],
								vel[0], vel[1], vel[2],
								rpy[0], rpy[1], rpy[2], 
								agv[0], agv[1], agv[2], 
								pos_ref[0], pos_ref[1], pos_ref[2], 
								rpy_ref[0], rpy_ref[1], rpy_ref[2], 
								vel_ref[0], vel_ref[1], vel_ref[2],
								torque[0], torque[1], torque[2],
								w[0],w[1],w[2],w[3],
								u[0],u[1],u[2],u[3],
								mreq[0],mreq[1],mreq[2],mreq[3],
								thrust[0],thrust[1],thrust[2],thrust[3],
								thrust_ref,
								rpy_vicon[0],rpy_vicon[1],rpy_vicon[2],
								acc[0],acc[1],acc[2],
								att_received[0],att_received[1],att_received[2],])
								# roll_torque])

	def conv(self, s):
		try:
			s=float(s)
		except ValueError:
			pass
		return s

	def openCSVMatrix(self, filename = 'filename'):
		rows = []
		with open(filename) as csvfile:
			reader = csv.reader(csvfile, delimiter=',')
			for row in reader:
				newrow = []
				for cell in row:
					newcell = self.conv(cell)
					newrow.append(newcell)
				rows.append(newrow)
		M = np.mat(rows)
		return M

	def savelog(self):
		# print(self.log_memory)
		try:
			# Create target Directory
			os.mkdir(self.folder_name)
			print("Directory \"" + self.folder_name +  "\" Created ") 
		except:
			print("Directory " + self.folder_name +  " already exists")
			pass
			
		with open(self.folder_name + '/log_' + time.strftime("%m%d_%H%M%S") + '.txt', 'w') as csvfile:
			writer = csv.writer(csvfile)
			writer.writerows(self.log_memory)
			print("CSV file: " + "log_" + time.strftime("%m%d_%H%M%S") + ".txt" + " created")

	def plot(self):
		# u_ILC = self.openCSVMatrix(filename = './ILC_input/ILC_input.txt')

		n = len(self.log_memory)
		log_memory_array = np.asarray(self.log_memory[1:n])
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
		x_torque = log_memory_array[:, i]; i+=1
		y_torque = log_memory_array[:, i]; i+=1
		z_torque = log_memory_array[:, i]; i+=1
		w0 = log_memory_array[:,i]; i+=1
		w1 = log_memory_array[:,i]; i+=1
		w2 = log_memory_array[:,i]; i+=1
		w3 = log_memory_array[:,i]; i+=1
		u0 = log_memory_array[:,i]; i+=1
		u1 = log_memory_array[:,i]; i+=1
		u2 = log_memory_array[:,i]; i+=1
		u3 = log_memory_array[:,i]; i+=1
		m0req = log_memory_array[:,i]; i+=1
		m1req = log_memory_array[:,i]; i+=1
		m2req = log_memory_array[:,i]; i+=1
		m3req = log_memory_array[:,i]; i+=1
		thrust0 = log_memory_array[:,i]; i+=1
		thrust1 = log_memory_array[:,i]; i+=1
		thrust2 = log_memory_array[:,i]; i+=1
		thrust3 = log_memory_array[:,i]; i+=1
		thrust_ref = log_memory_array[:,i]; i+=1
		roll_vicon = log_memory_array[:,i]; i+=1
		pitch_vicon = log_memory_array[:,i]; i+=1
		yaw_vicon = log_memory_array[:,i]; i+=1
		acc_x = log_memory_array[:,i]; i+=1
		acc_y = log_memory_array[:,i]; i+=1
		acc_z = log_memory_array[:,i]; i+=1
		roll_received = log_memory_array[:,i]; i+=1
		pitch_received = log_memory_array[:,i]; i+=1
		yaw_received = log_memory_array[:,i]; i+=1
		# roll_torque = log_memory_array[:,i]; i+=1
		# plt.subplot(5,1,1)
		# plt.plot(timestamp,vel_x,'r',timestamp,x_vel_ref,'r--',timestamp,vel_y,'g',timestamp,y_vel_ref,'g--',timestamp,vel_z, 'b',timestamp,z_vel_ref,'b--')
		# plt.legend(["x","x_ref","y","y_ref","z","z_ref"])
		# plt.ylabel('vel')
		# plt.grid(True)
		# plt.subplot(5,1,2)
		# plt.plot(timestamp,agv_x,'r',timestamp,agv_y,'g',timestamp,agv_z, 'b')
		# plt.legend(["x","y","z"])
		# plt.ylabel('agv')
		# plt.grid(True)
		plt.figure(1)
		plt.subplot(4,1,1)
		plt.plot(timestamp,pos_x,'r',timestamp,x_ref,'r--',timestamp,pos_y,'g',timestamp,y_ref,'g--',timestamp,pos_z, 'b', timestamp, z_ref, 'b--')
		plt.ylabel('pos')
		plt.legend(["x","x_ref","y","y_ref","z","z_ref"])
		plt.grid(True)
		plt.subplot(4,1,2)
		plt.plot(timestamp,roll,'r',timestamp,roll_ref,'r--',timestamp,pitch,'g',timestamp,pitch_ref,'g--',timestamp,yaw, 'b',timestamp,yaw_ref,'b--')#,timestamp,roll_vicon,'r-.',timestamp,pitch_vicon,'g-.',timestamp,yaw_vicon,'b-.')
		plt.legend(["roll","roll_ref","pitch","pitch_ref","yaw","yaw_ref"])#,"roll_vicon","pitch_vicon","yaw_vicon"])
		plt.ylabel('rpy')
		plt.grid(True)
		plt.subplot(4,1,3)
		plt.plot(timestamp,x_torque,'r',timestamp,y_torque,'g',timestamp,z_torque,'b')
		plt.legend(["roll","pitch","yaw"])
		plt.ylabel('torque')
		plt.grid(True)
		# plt.subplot(4,1,4)
		# plt.plot(timestamp,thrust_ref,'r')
		# plt.legend(["thrust_ref"])
		# plt.ylabel('thrust')
		# plt.grid(True)
		# plt.subplot(4,1,4)
		# plt.plot(timestamp,vel_x,'r',timestamp,vel_y,'g',timestamp,vel_z,'b')
		# plt.legend(["vel_x","vel_y","vel_z"])
		# plt.ylabel('vel')
		# plt.grid(True)
		plt.subplot(4,1,4)
		plt.plot(timestamp,w0,'r',timestamp,w1,'g',timestamp,w2,'b--',timestamp,w3,'k--')
		plt.legend(["w0","w1","w2","w3"])
		plt.ylabel('motorPwm')
		plt.grid(True)
		# plt.subplot(5,2,8)
		# plt.plot(timestamp,u0,'r',timestamp,u1,'g',timestamp,u2,'b--',timestamp,u3,'k--')
		# plt.legend(["u0","u1","u2","u3"])
		# plt.ylabel('motorForces')
		# plt.grid(True)
		# plt.subplot(5,2,9)
		# plt.plot(timestamp,m0req,'r',timestamp,m1req,'g',timestamp,m2req,'b--',timestamp,m3req,'k--')
		# plt.legend(["m0req","m1req","m2req","m3req"])
		# plt.ylabel('motorThrustBatCompUncapped')
		# plt.grid(True)
		# plt.subplot(5,2,10)
		# plt.plot(timestamp,thrust0,'r',timestamp,thrust1,'g',timestamp,thrust2,'b--',timestamp,thrust3,'k--')
		# plt.legend(["thrust0","thrust1","thrust2","thrust3"])
		# plt.ylabel('motorThrustUncapped')
		# plt.grid(True)
		plt.figure(2)
		plt.subplot(3,1,1)
		plt.plot(timestamp,vel_x,'r',timestamp,x_vel_ref,'r--',timestamp,vel_y,'g',timestamp,y_vel_ref,'g--',timestamp,vel_z, 'b',timestamp,z_vel_ref,'b--')
		plt.legend(["x","x_ref","y","y_ref","z","z_ref"])
		plt.ylabel('vel')
		plt.grid(True)
		plt.subplot(3,1,2)
		plt.plot(timestamp,thrust_ref,'r')
		plt.legend(["thrust_ref"])
		plt.ylabel('thrust ref')
		plt.grid(True)
		plt.subplot(3,1,3)
		plt.plot(timestamp,acc_x,'r',timestamp,acc_y,'g',timestamp,acc_z, 'b')
		plt.legend(["x","y","z"])
		plt.ylabel('acc')
		plt.grid(True)
		# plt.subplot(4,1,4)
		# plt.plot(roll,roll_torque,'r')
		# plt.legend(["roll_torque"])
		# plt.ylabel('roll_torque')
		# plt.xlabel('roll')
		# plt.grid(True)
		plt.figure()
		plt.plot(timestamp,roll_vicon,'r',timestamp,pitch_vicon,'g',timestamp,yaw_vicon,'b')
		plt.legend(["roll","pitch","yaw"])
		plt.xlabel('time')
		plt.ylabel('frame attitude')
		plt.grid(True)
		plt.figure()
		plt.plot(timestamp,agv_x,'r',timestamp,agv_y,'g',timestamp,agv_z, 'b')
		plt.legend(["x","y","z"])
		plt.ylabel('agv')
		plt.grid(True)

		plt.figure()
		plt.plot(timestamp,roll_received,'r',timestamp,pitch_received,'g--',timestamp,yaw_received,'b')
		plt.legend(["rollPID","pitchPID","yawPID"])
		plt.ylabel('PID')
		plt.grid(True)


		
		plt.show()