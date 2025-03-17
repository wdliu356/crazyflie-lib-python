"""
Main program of ground controller

Using multiprocessing module. Runs on a multicore PC (>= 3)

Change log folder name master.folder_name as you wish

User inputs:
	refer to gimbal_platform_keyboard.py for keyboard control
	Press ESC for emergency stop
"""
import logging
import time
import os
import sys
import csv
import numpy as np
import math
import multiprocessing
from threading import Thread
import matplotlib.pyplot as plt

# from cflib.crazyflie.log import LogConfig
import cflib
# from tracker_thread_backup import Tracker
from cflib.crazyflie import Crazyflie

from four_crazyflie_single import SingleCF
# from crazyflie_imu import IMUCF
from gimbal_platform_logger import CombinedLogger
from gimbal_platform_keyboard import KeyboardControl
# from tcp_server_modify import Vicon
from tcp_server_modify_2 import Vicon
from test_write_txt import write_txt_file

logging.basicConfig(level=logging.ERROR)

roll_all = np.array([0.0])
roll_torque_all = np.array([0.0])


class CombinedLogger:
	
	def __init__(self, folder_name='log_files'):
		self.folder_name = folder_name
		self.log_memory = [["timestamp", "dt",
							"roll","pitch","yaw"
							"w0","w1","w2","w3",
							"thrust_ref",
							"roll_torque"
							]] # error rotation integration

	def log_append(self, timestamp, dt, 
						 roll, pitch,yaw,
						 w,
						 thrust_ref,
						 roll_torque):
		"""
		timestamp = round(timestamp,3)
		e = round(e,3)
		d = round(d,3)
		"""
		self.log_memory.append([timestamp, dt,
								roll, pitch,yaw,
								w[0],w[1],w[2],w[3],
								thrust_ref,
								roll_torque])

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
		roll = log_memory_array[:, i]; i+=1
		pitch = log_memory_array[:, i]; i+=1
		yaw = log_memory_array[:, i]; i+=1
		w0 = log_memory_array[:,i]; i+=1
		w1 = log_memory_array[:,i]; i+=1
		w2 = log_memory_array[:,i]; i+=1
		w3 = log_memory_array[:,i]; i+=1
		thrust_ref = log_memory_array[:,i]; i+=1
		roll_torque = log_memory_array[:,i]; i+=1
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
		plt.subplot(3,1,1)
		plt.plot(timestamp,roll,'r')#,timestamp,roll_vicon,'r-.',timestamp,pitch_vicon,'g-.',timestamp,yaw_vicon,'b-.')
		plt.legend(["roll"])
		plt.ylabel('roll')
		plt.grid(True)
		plt.subplot(3,1,2)
		plt.plot(timestamp,w0,'r',timestamp,w1,'g',timestamp,w2,'b--',timestamp,w3,'k--')
		plt.legend(["w0","w1","w2","w3"])
		plt.ylabel('motorPwm')
		plt.grid(True)
		plt.subplot(3,1,3)
		plt.plot(roll,roll_torque)
		plt.xlabel('roll')
		plt.ylabel('roll_torque')
		plt.grid(True)
		plt.figure(2)
		plt.plot(timestamp,roll,'r',timestamp,pitch,'g',timestamp,yaw,'b')
		plt.xlabel('time')
		plt.ylabel('angle')
		
		plt.show()


class CombinedFrame:
	def __init__(self):
		self.qc_setup_time = 0.25
		self.update_rate = 0.01

		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.dt = 0


	def run(self, stop_shared,start,state_fb_shared,thrust_shared,torque_mode,torque_shared):

		# Initialize the low-level drivers (don't list the debug drivers)
		cflib.crtp.init_drivers(enable_debug_driver=False)
		# Scan for Crazyflies and use the first one found
		print('Scanning interfaces for Crazyflies...')
		available = cflib.crtp.scan_interfaces()
		print('Crazyflies found:')
		for i in available:
			print(i[0])
		time.sleep(0.25)

		self.qc = SingleCF('radio://0/80/2M/E7E7E7E7E7', 0)
		# self.qc = SingleCF('radio://0/100/2M/E7E7E7E7E3', 0)
		time.sleep(self.qc_setup_time)


		while 1:
			if stop_shared.value == 1:
				print('stop')
				self.stop()
				print('stop')
				break
			self.current_time = time.time()
			# print(self.current_time)
			if self.current_time - self.last_loop_time > self.update_rate:
				self.qc.thrustd = thrust_shared.value
				self.qc.torque_mode = torque_mode.value
				self.qc.torque_shared = torque_shared.value
				self.qc.start = start.value
				self.qc._torque_test()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				self.log.append(self.dt)

				state_fb_shared[:] = [self.qc.roll,self.qc.pitch,self.qc.yawfb,
						  self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,torque_shared.value
						  ]
			else:
				time.sleep(0.001)

		self.stop()

	def stop(self):
		self.qc._stop_crazyflie()



class Master:
	def __init__(self):
		self.start_time = 0
		self.stop_shared = multiprocessing.Value('i',0)
		self.thrust_shared = multiprocessing.Value('f',0)
		self.start = multiprocessing.Value('i',0)
		self.start.value = False
		self.torque_mode = multiprocessing.Value('i',0)
		self.torque_mode.value = False
		self.torque_shared = multiprocessing.Value('f',0)
		self.torque_shared.value = 0
		self.state_fb_shared = multiprocessing.Array('f',8)

		self.keyboard = KeyboardControl(mode = 0,ground_mode=True)
		self.g = 9.81
		# self.m = 0.083 #for small
		self.m = 0.415 #for big
		self.thrust_shared.value = self.m*self.g*0.5
		self.logger = CombinedLogger(folder_name='torque_test')

		self.cbframe = CombinedFrame()
		self.p_cbframe = multiprocessing.Process(target=self.cbframe.run, 
												args=(self.stop_shared, self.start,self.state_fb_shared, self.thrust_shared,self.torque_mode,self.torque_shared))

		time.sleep(0.25)
		
	def run(self):
		self.start_time = time.time()
		self.last_loop_time = self.start_time
		print(self.start_time)
		loop_num = 0
		self.keyboard.torque_mode = True
		self.op = Vicon()
		global roll_all,roll_torque_all
		while 1:
			loop_num+=1
			# print('in master',self.start.value)
			if self.keyboard.stop == 1:
				self.stop_shared.value = 1
				break

			current_time = time.time()
			quat = self.op.rotation
			roll_v = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))
			pitch_v = np.arcsin(2*(quat[0]*quat[2]-quat[3]*quat[1]))
			yawfb_v = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))
				
			if current_time - self.last_loop_time > 0.01:
				self.keyboard.command.update()
				self.torque_mode.value = self.keyboard.torque_mode
				self.torque_shared.value = self.keyboard.roll_torque
				self.start.value = self.keyboard.start
				self.stop_shared.value = self.keyboard.stop
				self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
										roll_v,pitch_v,yawfb_v,
										[self.state_fb_shared[1],self.state_fb_shared[2],self.state_fb_shared[3],self.state_fb_shared[4]],
										self.thrust_shared.value,
										self.torque_shared.value
										)
				self.last_loop_time = current_time
				roll_all = np.append(roll_all,self.state_fb_shared[0])
				roll_torque_all = np.append(roll_torque_all,self.state_fb_shared[5])
			else:
				time.sleep(0.0001)
		
		slope = np.polyfit(roll_all,roll_torque_all,1)
		print('slope',slope)
		

		self.stop()

	def stop(self):
		self.stop_shared.value = 1
		self.keyboard.command.quit()
		self.keyboard.command.destroy()
		self.logger.savelog()
		self.logger.plot()
		# print(self.debug1_shared[:])
		time.sleep(0.1)
		# print(self.debug2_shared[:])


if __name__ == '__main__':
	master = Master()
	master.p_cbframe.start()
	time.sleep(9)
	master.run()

	master.p_cbframe.join()

