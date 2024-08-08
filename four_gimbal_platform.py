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


class CombinedFrame:
	def __init__(self):
		self.qc_setup_time = 0.25
		self.update_rate = 0.01

		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.dt = 0

	# def run(self, stop_shared, vel_ref_shared, yaw_ref_shared, yaw_rate_shared,reached,groundmode,state_fb_shared):

	# 	# Initialize the low-level drivers (don't list the debug drivers)
	# 	cflib.crtp.init_drivers(enable_debug_driver=False)
	# 	# Scan for Crazyflies and use the first one found
	# 	print('Scanning interfaces for Crazyflies...')
	# 	available = cflib.crtp.scan_interfaces()
	# 	print('Crazyflies found:')
	# 	for i in available:
	# 		print(i[0])
	# 	time.sleep(0.25)

	# 	self.qc = SingleCF('radio://0/80/2M/E7E7E7E7E7', 0)
	# 	# self.qc = SingleCF('radio://0/100/2M/E7E7E7E7E3', 0)
	# 	time.sleep(self.qc_setup_time)


	# 	while 1:
	# 		if stop_shared.value == 1:
	# 			print('stop')
	# 			self.stop()
	# 			print('stop')
	# 			break
	# 		self.current_time = time.time()
	# 		if self.current_time - self.last_loop_time > self.update_rate:
	# 			self.qc.vx = vel_ref_shared[0]
	# 			self.qc.vy = vel_ref_shared[1]
	# 			self.qc.vz = vel_ref_shared[2]
	# 			self.qc.yaw = yaw_ref_shared[0]
	# 			self.qc.yawrate = yaw_rate_shared[0] 
	# 			self.qc.groundmode = groundmode
	# 			self.qc.reset = reached
	# 			self.qc._update_motors()
	# 			self.dt = self.current_time - self.last_loop_time
	# 			self.last_loop_time = self.current_time
	# 			self.log.append(self.dt)

	# 			state_fb_shared[:] = [self.qc.vxfb, self.qc.vyfb, self.qc.vzfb, self.qc.rollratefb, self.qc.pitchratefb, self.qc.yawratefb, self.qc.roll, self.qc.pitch, self.qc.yawfb, self.qc.thrust,self.qc.rollref,
	# 					  self.qc.pitchref,self.qc.yawref,self.qc.rolltorque,self.qc.pitchtorque,self.qc.yawtorque,self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,self.qc.u1,self.qc.u2,self.qc.u3,self.qc.u4,
	# 					  self.qc.m1req,self.qc.m2req,self.qc.m3req,self.qc.m4req,self.qc.thrust1,self.qc.thrust2,self.qc.thrust3,self.qc.thrust4]
	# 		else:
	# 			time.sleep(0.001)

	# 	self.stop()

	def run(self, stop_shared, att_ref_shared, yaw_rate_shared,reached,start,state_fb_shared):

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
			if self.current_time - self.last_loop_time > self.update_rate:
				self.qc.rolld = att_ref_shared[0]
				self.qc.pitchd = att_ref_shared[1]
				self.qc.yawd = att_ref_shared[2]
				# self.qc.yaw = yaw_ref_shared[0]
				self.qc.yawrate = yaw_rate_shared[0] 
				self.qc.start = start
				self.qc.reset = reached
				self.qc._update_motors()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				self.log.append(self.dt)

				state_fb_shared[:] = [self.qc.vxfb, self.qc.vyfb, self.qc.vzfb, self.qc.rollratefb, self.qc.pitchratefb, self.qc.yawratefb, self.qc.roll, self.qc.pitch, self.qc.yawfb, self.qc.thrust,self.qc.rollref,
						  self.qc.pitchref,self.qc.yawref,self.qc.rolltorque,self.qc.pitchtorque,self.qc.yawtorque,self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,self.qc.u1,self.qc.u2,self.qc.u3,self.qc.u4,
						  self.qc.m1req,self.qc.m2req,self.qc.m3req,self.qc.m4req,self.qc.thrust1,self.qc.thrust2,self.qc.thrust3,self.qc.thrust4]
			else:
				time.sleep(0.001)

		self.stop()

	def stop(self):
		self.qc._stop_crazyflie()



class Controller:
	def __init__(self):
		# self.qc_setup_time = 0.25
		self.dt = 0.01
		self.mode = 0 # 0 for position, 1 for velocity
		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.dt = 0.01
		self.perr = np.array([0.0,0.0,0.0])
		self.verr = np.array([0.0,0.0,0.0])
		self.pKp = np.array([0.3,0.3,0.3])
		self.pKi = np.array([0.01,0.01,0.01])
		self.reached = False

	def position_control(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reached):
		self.perr += (np.array(pos_ref_shared) - np.array(pos_fb_shared)) * self.dt
		for i in range(3):
			if abs(self.perr[i]) > 0.5:
				self.perr[i] = 0.5*np.sign(self.perr[i])
		vel_ref_shared[:] = self.pKp * (np.array(pos_ref_shared) - np.array(pos_fb_shared)) + self.pKi * self.err
		if self.mode == 0:
			if np.linalg.norm(np.array(pos_ref_shared) - np.array(pos_fb_shared)) < 0.01:
				reached.value = True
			else:
				reached.value = False

	def velocity_control(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reached):
		vel_ref_shared[:] = pos_ref_shared
		if np.linalg.norm(np.array(vel_ref_shared) - np.array(vel_fb_shared)) < 0.1:
			reached.value = True
		else:
			reached.value = False

	def run(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reached, stop_shared):
		while 1:
			if stop_shared.value == 1:
				break
			self.current_time = time.time()
			if self.current_time - self.last_loop_time > self.dt:
				# self.err += (np.array(pos_ref_shared) - np.array(pos_fb_shared)) * self.dt
				# vel_ref_shared[:] = self.Kp * (np.array(pos_ref_shared) - np.array(pos_fb_shared)) + self.Ki * self.err
				# self.last_loop_time = self.current_time
				# if np.linalg.norm(np.array(pos_ref_shared) - np.array(pos_fb_shared)) < 0.1:
				# 	reached.value = 1
				# else:
				# 	reached.value = 0

				vel_ref_shared[:] = [0,0,0]
				if np.linalg.norm(np.array(vel_ref_shared) - np.array(vel_fb_shared)) < 0.1:
					reached.value = True
				else:
					reached.value = False
			else:
				time.sleep(0.001)

		self.stop()
	def stop(self):
		pass

class Master:
	def __init__(self):
		self.start_time = 0
		self.vel_ref_shared = multiprocessing.Array('f',3)
		self.yaw_ref_shared = multiprocessing.Array('f',1)
		self.yaw_rate_shared = multiprocessing.Array('f',1)
		self.state_fb_shared = multiprocessing.Array('f',32)
		self.reached = multiprocessing.Value('i',0)
		self.groundmode = multiprocessing.Value('i',0)
		self.groundmode.value = True
		self.pos_ref_shared = multiprocessing.Array('f',3)
		self.pos_fb_shared = multiprocessing.Array('f',3)
		self.vel_fb_shared = multiprocessing.Array('f',3)
		# self.vel_shared = multiprocessing.Array('f',3)
		# self.quat_shared = multiprocessing.Array('f',4)
		# self.omega_shared = multiprocessing.Array('f',3)
		# self.quat_shared[:] = [1,0,0,0]
		# self.alpha_shared = multiprocessing.Array('f',4)
		# self.beta_shared = multiprocessing.Array('f',4)
		# self.thrust_shared = multiprocessing.Array('f',4)
		# self.debug1_shared = multiprocessing.Array('f',15) # control
		# self.debug2_shared = multiprocessing.Array('f',16) # combinedframe
		self.stop_shared = multiprocessing.Value('i',0)

		self.keyboard = KeyboardControl(control_mode='attitude')
		self.logger = CombinedLogger(folder_name='log_test_0801')

		self.controller = Controller()
		self.p_control = multiprocessing.Process(target=self.controller.run, 
												args=(self.pos_ref_shared,self.pos_fb_shared, self.vel_fb_shared, self.vel_ref_shared, self.reached, self.stop_shared))
		self.cbframe = CombinedFrame()
		self.p_cbframe = multiprocessing.Process(target=self.cbframe.run, 
												args=(self.stop_shared, self.vel_ref_shared, self.yaw_ref_shared, self.yaw_rate_shared, self.reached, self.groundmode, self.state_fb_shared))

		# self.op = Vicon()
		time.sleep(0.25)
		
	def run(self):
		self.start_time = time.time()
		self.last_loop_time = self.start_time
		print(self.start_time)
		while 1:
			if self.keyboard.stop == 1:
				self.stop_shared.value = 1
				break
			current_time = time.time()
			if current_time - self.last_loop_time > 0.005:
				# self.pos_fb_shared[:] = self.op.position
				self.pos_fb_shared[:] = [0.0,0.0,0.0]
				self.keyboard.command.update()
				self.stop_shared.value = self.keyboard.stop
				self.pos_ref_shared[:] = self.keyboard.pos
				self.yaw_ref_shared = self.keyboard.rpy[2]
				self.vel_ref_shared[:] = self.keyboard.vel
				self.yaw_rate_shared = self.keyboard.agv[2]
				self.vel_fb_shared[:] = self.state_fb_shared[0:3]
				self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
									   self.pos_fb_shared[:], 
									   self.state_fb_shared[0:3], self.state_fb_shared[6:9], self.state_fb_shared[3:6],
									   self.pos_ref_shared[:], self.state_fb_shared[10:13],
									   self.vel_ref_shared[:], self.state_fb_shared[13:16],self.state_fb_shared[16:20],self.state_fb_shared[20:24],self.state_fb_shared[24:28],self.state_fb_shared[28:32],self.state_fb_shared[9])
				self.last_loop_time = current_time
			else:
				time.sleep(0.0001)

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
	master.p_control.start()
	master.run()

	master.p_cbframe.join()
	master.p_control.join()

