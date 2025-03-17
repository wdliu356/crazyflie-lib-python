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
							"x_ref", "y_ref", "z_ref",
							"vel_x", "vel_y", "vel_z",
							"x_vel_ref", "y_vel_ref", "z_vel_ref",
							"roll", "pitch", "yaw",
							"roll_ref", "pitch_ref", "yaw_ref",
							"rollPID", "pitchPID", "yawPID",
							"agv_x", "agv_y", "agv_z",
							"x_torque", "y_torque", "z_torque",
							"w0","w1","w2","w3",
							"rollD","pitchD","yawD",
							]]


	def log_append(self, timestamp, dt,
				pos,pos_ref,vel,vel_ref,rpy,rpy_ref,rpyPID,agv,torque,w,rpyd):#,roll_torque):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		d = round(d,3)
		"""
		self.log_memory.append([timestamp, dt,
								pos[0], pos[1], pos[2],
								pos_ref[0], pos_ref[1], pos_ref[2],
								vel[0], vel[1], vel[2],
								vel_ref[0], vel_ref[1], vel_ref[2],
								rpy[0], rpy[1], rpy[2],
								rpy_ref[0], rpy_ref[1], rpy_ref[2],
								rpyPID[0], rpyPID[1], rpyPID[2],
								agv[0], agv[1], agv[2],
								torque[0], torque[1], torque[2],
								w[0],w[1],w[2],w[3],
								rpyd[0],rpyd[1],rpyd[2]
								])#,

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
		pos = np.array([pos_x,pos_y,pos_z])
		pos_ref_x = log_memory_array[:, i]; i+=1
		pos_ref_y = log_memory_array[:, i]; i+=1
		pos_ref_z = log_memory_array[:, i]; i+=1
		pos_ref = np.array([pos_ref_x,pos_ref_y,pos_ref_z])
		vel_x = log_memory_array[:, i]; i+=1
		vel_y = log_memory_array[:, i]; i+=1
		vel_z = log_memory_array[:, i]; i+=1
		vel = np.array([vel_x,vel_y,vel_z])
		vel_ref_x = log_memory_array[:, i]; i+=1
		vel_ref_y = log_memory_array[:, i]; i+=1
		vel_ref_z = log_memory_array[:, i]; i+=1
		vel_ref = np.array([vel_ref_x,vel_ref_y,vel_ref_z])
		roll = log_memory_array[:, i]; i+=1
		pitch = log_memory_array[:, i]; i+=1
		yaw = log_memory_array[:, i]; i+=1
		rpy = np.array([roll,pitch,yaw])
		roll_ref = log_memory_array[:, i]; i+=1
		pitch_ref = log_memory_array[:, i]; i+=1
		yaw_ref = log_memory_array[:, i]; i+=1
		rpy_ref = np.array([roll_ref,pitch_ref,yaw_ref])
		rollPID = log_memory_array[:, i]; i+=1
		pitchPID = log_memory_array[:, i]; i+=1
		yawPID = log_memory_array[:, i]; i+=1
		rpyPID = np.array([rollPID,pitchPID,yawPID])
		agv_x = log_memory_array[:, i]; i+=1
		agv_y = log_memory_array[:, i]; i+=1
		agv_z = log_memory_array[:, i]; i+=1
		agv = np.array([agv_x,agv_y,agv_z])
		x_torque = log_memory_array[:, i]; i+=1
		y_torque = log_memory_array[:, i]; i+=1
		z_torque = log_memory_array[:, i]; i+=1
		torque = np.array([x_torque,y_torque,z_torque])
		w0 = log_memory_array[:,i]; i+=1
		w1 = log_memory_array[:,i]; i+=1
		w2 = log_memory_array[:,i]; i+=1
		w3 = log_memory_array[:,i]; i+=1
		w = np.array([w0,w1,w2,w3])
		rollD = log_memory_array[:,i]; i+=1
		pitchD = log_memory_array[:,i]; i+=1
		yawD = log_memory_array[:,i]; i+=1
		rpyD = np.array([rollD,pitchD,yawD])


		plt.figure(1)
		plt.subplot(3,1,1)
		plt.plot(timestamp,roll,'r')
		plt.plot(timestamp,rollD,'r--')
		plt.plot(timestamp,pitch,'g')
		plt.plot(timestamp,pitchD,'g--')
		plt.plot(timestamp,yaw,'b')
		plt.plot(timestamp,yawD,'b--')
		plt.legend(["roll","roll_ref","pitch","pitch_ref","yaw","yaw_ref"])
		plt.ylabel('angle')
		plt.grid(True)
		plt.subplot(3,1,2)
		plt.plot(timestamp,w0,'r')
		plt.plot(timestamp,w1,'g')
		plt.plot(timestamp,w2,'b')
		plt.plot(timestamp,w3,'k')
		plt.legend(["m0","m1","m2","m3"])
		plt.ylabel('motorPwm')
		plt.grid(True)
		plt.subplot(3,1,3)
		plt.plot(timestamp,x_torque,'r')
		plt.plot(timestamp,y_torque,'g')
		plt.plot(timestamp,z_torque,'b')
		plt.legend(["x_torque","y_torque","z_torque"])
		plt.ylabel('torque')
		plt.grid(True)
		plt.figure(2)
		plt.plot(timestamp,pos_x,'r')
		plt.plot(timestamp,pos_ref_x,'r--')
		plt.plot(timestamp,pos_y,'g')
		plt.plot(timestamp,pos_ref_y,'g--')
		plt.plot(timestamp,pos_z,'b')
		plt.plot(timestamp,pos_ref_z,'b--')
		plt.legend(["x","x_ref","y","y_ref","z","z_ref"])
		plt.ylabel('pos')
		plt.grid(True)
		plt.figure(3)
		plt.plot(timestamp,vel_x,'r')
		plt.plot(timestamp,vel_ref_x,'r--')
		plt.plot(timestamp,vel_y,'g')
		plt.plot(timestamp,vel_ref_y,'g--')
		plt.plot(timestamp,vel_z,'b')
		plt.plot(timestamp,vel_ref_z,'b--')
		plt.legend(["x","x_ref","y","y_ref","z","z_ref"])
		plt.ylabel('vel')
		plt.grid(True)
		plt.figure(4)
		plt.plot(timestamp,agv_x,'r')
		plt.plot(timestamp,agv_y,'g')
		plt.plot(timestamp,agv_z,'b')
		plt.legend(["roll","pitch","yaw"])
		plt.ylabel('agv')
		plt.grid(True)
		plt.figure(5)
		plt.plot(timestamp,rollPID,'r')
		plt.plot(timestamp,pitchPID,'g')
		plt.plot(timestamp,yawPID,'b')
		plt.legend(["rollPID","pitchPID","yawPID"])
		plt.ylabel('PID')
		plt.grid(True)
		
		plt.show()