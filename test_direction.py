import logging
import time
import os
import sys
import csv
import numpy as np
import math
import multiprocessing
from threading import Thread
import cflib
from cflib.crazyflie import Crazyflie
from four_crazyflie_single import SingleCF
import matplotlib.pyplot as plt
from gimbal_platform_keyboard import KeyboardControl
from tcp_server_modify_2 import Vicon

class CombinedFrame:
	def __init__(self):
		self.qc_setup_time = 0.25
		self.update_rate = 0.005

		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.dt = 0
		
		self.rpy_prev = np.array([0.0,0.0,0.0])
		self.rpy_rate_prev = np.array([0.0,0.0,0.0])


	def run(self, stop_shared, att_ref_shared, yaw_rate_shared,start,thrust_shared,groundmode,loc_mode,frame_roll,state_shared):

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
		time.sleep(self.qc_setup_time)
		self.op = Vicon()
		for i in range(100):
			quat = self.op.body_rotation
			yaw_start = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))
			time.sleep(0.01)

		while 1:
			if stop_shared.value == 1:
				print('stop')
				self.stop()
				print('stopped')
				break
			self.current_time = time.time()
			
			# print(self.current_time)
			if self.current_time - self.last_loop_time > self.update_rate:
				quat = self.op.body_rotation
				self.qc.roll_v = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))
				self.qc.pitch_v = np.arcsin(2*(quat[0]*quat[2]-quat[3]*quat[1]))
				self.qc.yawfb_v = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))-yaw_start
				print('roll,pitch,yawfb',self.qc.roll_v,self.qc.pitch_v,self.qc.yawfb_v)
				if self.dt == 0:
					self.qc.rollratefb_v = 0
					self.qc.pitchratefb_v = 0
					self.qc.yawratefb_v = 0 
				else:
					# self.qc.rollratefb_v = self.rpy_rate_prev[0]*0.8 + 0.2*(self.qc.roll_v-self.rpy_prev[0])/self.dt
					# self.qc.pitchratefb_v = self.rpy_rate_prev[1]*0.8 + 0.2*(self.qc.pitch_v-self.rpy_prev[1])/self.dt
					# self.qc.yawratefb_v = self.rpy_rate_prev[2]*0.8 + 0.2*(self.qc.yawfb_v-self.rpy_prev[2])/self.dt
					self.qc.rollratefb_v = self.op.body_rotation_rate[0]
					self.qc.pitchratefb_v = self.op.body_rotation_rate[1]
					self.qc.yawratefb_v = self.op.body_rotation_rate[2]
				self.rpy_prev = np.array([self.qc.roll_v,self.qc.pitch_v,self.qc.yawfb_v])
				self.rpy_rate_prev = np.array([self.qc.rollratefb_v,self.qc.pitchratefb_v,self.qc.yawratefb_v])

				self.qc.rolld = att_ref_shared[0]
				self.qc.pitchd = att_ref_shared[1]
				self.qc.yawd = att_ref_shared[2]
				self.qc.thrustd = thrust_shared
				self.qc.groundmode = groundmode
				# self.qc.yaw = yaw_ref_shared[0]
				self.qc.yawrate = yaw_rate_shared[0] 
				self.qc.start = start.value
				self.qc.frame_roll = frame_roll
				# print('in cbframe',self.qc.start)
				self.qc.loc_mode = loc_mode
				# self.qc.yawfb_v = 0
				self.qc._direction_test()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				### ADD [self.dt,self.qc.rollPID,self.qc.pitchPID,self.qc.yawPID] to log as a row
				# self.log.append([self.dt,self.qc.rollPID,self.qc.pitchPID,self.qc.yawPID])
				state_shared[:]=np.array([self.dt,self.qc.rollPID,self.qc.pitchPID,self.qc.yawPID,self.qc.roll_v,self.qc.pitch_v,self.qc.yawfb_v,
							  self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,
							  self.qc.rolltorque,self.qc.pitchtorque,self.qc.yawtorque,
							  self.qc.rollratefb_v,self.qc.pitchratefb_v,self.qc.yawratefb_v])

				
			else:
				time.sleep(0.001)

		self.stop()

	def stop(self):
		self.qc._stop_crazyflie()



if __name__ == '__main__':
	print('Start initialization')
	cb=CombinedFrame()
	print('Initialized')
	start_time = time.time()
	keyboard = KeyboardControl(mode = 0,ground_mode=True)
	stop_shared = multiprocessing.Value('i', False)
	state_shared = multiprocessing.Array('d', 17)
	start_shared = multiprocessing.Value('i', False)
	cb_process = multiprocessing.Process(target=cb.run, args=(stop_shared,[0.0,0.0,0.0],[0.0],start_shared,0.4,True,False,0.0,state_shared))
	cb_process.start()
	last_loop_time = time.time()
	state_all = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0])
	while 1:
		current_time = time.time()
		if keyboard.stop == 1:
			stop_shared.value = 1
			break
		if current_time - last_loop_time > 0.05:
			keyboard.command.update()
			if keyboard.stop:
				stop_shared.value = 1
				break
			if keyboard.start:
				start_shared.value = 1
			last_loop_time = current_time
			## add state_shared to state_all
			state_all = np.vstack((state_all,np.array(state_shared[:])))
		else:
			time.sleep(0.01)
	plt.figure(1)
	plt.plot(state_all[:,1])
	plt.plot(state_all[:,2])
	plt.plot(state_all[:,3])
	plt.legend(['rollPID','pitchPID','yawPID'])
	plt.figure(2)
	plt.plot(state_all[:,4])
	plt.plot(state_all[:,5])
	plt.plot(state_all[:,6])
	plt.legend(['roll','pitch','yaw'])
	plt.figure(3)
	plt.plot(state_all[:,7])
	plt.plot(state_all[:,8],'-*')
	plt.plot(state_all[:,9],'--')
	plt.plot(state_all[:,10],'-.')
	plt.legend(['m1','m2','m3','m4'])
	plt.figure(4)
	plt.plot(state_all[:,11])
	plt.plot(state_all[:,12],'-*')
	plt.plot(state_all[:,13],'--')
	plt.legend(['rolltorque','pitchtorque','yawtorque'])
	plt.figure(5)
	plt.plot(state_all[:,14])
	plt.plot(state_all[:,15],'-*')
	plt.plot(state_all[:,16],'--')
	plt.legend(['rollratefb','pitchratefb','yawratefb'])
	plt.show()
	# plt.figure(1)
	# plt.plot(cb.qc.m1)
	# plt.plot(cb.qc.m2)
	# plt.plot(cb.qc.m3)
	# plt.plot(cb.qc.m4)
	# plt.legend(['m1','m2','m3','m4'])
	
