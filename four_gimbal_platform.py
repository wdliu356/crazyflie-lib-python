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


	def run(self, stop_shared, att_ref_shared, yaw_rate_shared,reached,start,state_fb_shared,thrust_shared,groundmode,loc_mode):

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
				self.qc.rolld = att_ref_shared[0]
				self.qc.pitchd = att_ref_shared[1]
				self.qc.yawd = att_ref_shared[2]
				self.qc.thrustd = thrust_shared.value
				self.qc.groundmode = groundmode.value
				# self.qc.yaw = yaw_ref_shared[0]
				self.qc.yawrate = yaw_rate_shared[0] 
				self.qc.start = start.value
				# print('in cbframe',self.qc.start.value)
				self.qc.reset = reached.value
				self.qc.loc_mode = loc_mode.value
				self.qc._update_motors()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				self.log.append(self.dt)

				state_fb_shared[:] = [self.qc.vxfb, self.qc.vyfb, self.qc.vzfb, self.qc.rollratefb, self.qc.pitchratefb, self.qc.yawratefb, self.qc.roll, self.qc.pitch, self.qc.yawfb, self.qc.thrust,self.qc.rollreceived,
						  self.qc.pitchreceived,self.qc.yawreceived,self.qc.rolltorque,self.qc.pitchtorque,self.qc.yawtorque,self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,self.qc.u1,self.qc.u2,self.qc.u3,self.qc.u4,
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
		self.pKp = np.array([5.0,-5.0,-15.0])# minus sign on y and z because of the coordinate system
		self.pKi = np.array([0.1,-0.1,-2.5])# minus sign on y and z because of the coordinate system
		self.pKd = np.array([3.5,3.5,3.0])
		self.reached = False
		self.groundmode = True
		self.g = 9.81
		self.z_factor = 0.7
		# self.m = 0.083 #for small
		self.m = 0.415 #for big
		self.roll_factor = 1.0
		self.pitch_factor = 1.0
		self.num = 0
		self.num_bad = 0 # TEST
	
	def acc_control(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reached, acc_ref_shared,att_ref_shared,att_fb,auto_mode,groundmode):
		# acc_ref_shared[:] = self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
		if auto_mode.value:
			if pos_fb_shared[2] > 0.02 or pos_ref_shared[2] > 0.02:
				if self.groundmode:
					self.groundmode = False
					groundmode.value = False
					print('airmode')
					self.perr = np.array([0.0,0.0,0.0])
					self.verr = np.array([0.0,0.0,0.0])
			else:
				if not self.groundmode:
					self.groundmode = True
					groundmode.value = True
					print('groundmode')
					self.perr = np.array([0.0,0.0,0.0])
					self.verr = np.array([0.0,0.0,0.0])
		else:
			self.groundmode = groundmode.value
			if self.groundmode:
				print('groundmode')
			else:
				print('airmode')
		if self.mode == 0: #tune here
			if self.groundmode:
				self.pKp = np.array([8.0,-8.0,0])
				self.pKi = np.array([1.2,-1.2,0])
				self.pKd = np.array([6.0,6.0,0])
				# self.pKp = np.array([7.0,-7.0,-0.0])
				# self.pKi = np.array([1.2,-1.2,-0.0])
				# self.pKd = np.array([5.0,5.0,0.0])
				# self.pKp = np.array([6.0,-6.0,-15.0])
				# self.pKi = np.array([0.2,-0.2,-2.5])
				# self.pKd = np.array([4.0,4.0,3.0])
			else:
				# for small
				# self.pKp = np.array([3.5,-3.5,-10.0])
				# self.pKi = np.array([0.35,-0.35,-3.5])
				# self.pKd = np.array([6.0,6.0,30.0])

				# for big
				self.pKp = np.array([10.0,-10.0,-80.0])
				self.pKi = np.array([1.5,-1.5,-60.0])
				self.pKd = np.array([5.0,5.0,30.0])
			self.perr += (np.array(pos_ref_shared) - np.array(pos_fb_shared)) * self.dt
			# for i in range(3):
			# 	if abs(self.perr[i]) > 1:
			# 		self.perr[i] = 1*np.sign(self.perr[i])
			acc_ref_shared[:] = self.pKp * (np.array(pos_ref_shared) - np.array(pos_fb_shared)) + self.pKi * self.perr + self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
			if self.groundmode:
				
				if np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2) < 0.001 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.05:
					reached.value = True
					self.z_factor = 0.4
				elif np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2) < 1.5 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.2:
					reached.value = False
					x = np.max([(np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2)-0.001)/1.499,(np.abs(att_ref_shared[2]+att_fb[2])-0.05)/0.15])
					# self.z_factor = 0.4 + 0.5*x
					# self.z_factor = 0.4 + 0.5*np.log2(1+x) ##seems to be good
					self.z_factor = 0.4 - 0.5 * x * (x-2)##seems to be the best
					self.roll_factor = 1.0# for linear
					# self.roll_factor = 1.3# for log
				else:
					reached.value = False
					self.z_factor = 0.9
					self.roll_factor = 1.0# for linear
					# self.roll_factor = 1.3# for log

				# print('acc_diff,yaw_diff,z_factor',[np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2),np.abs(att_ref_shared[2]-att_fb[2]),self.z_factor])
				# print('yaw_fb,yaw_ref',[att_fb[2],att_ref_shared[2]])
					
		else:
			acc_ref_shared[:] = self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
			# if self.groundmode:
			# 	if np.linalg.norm(np.array(vel_ref_shared) - np.array(vel_fb_shared)) < 0.01:
			# 		reached.value = True
			# 	else:
			# 		reached.value = False

	def target_setup(self,acc_ref_shared,att_ref_shared,thrust_shared,loc_mode):
		yawd = att_ref_shared[2]
		a = np.sin(yawd)
		b = np.cos(yawd)
		if self.groundmode:
			if not loc_mode.value:
				pitch_d = self.pitch_factor*np.arctan(-(b*acc_ref_shared[0]+a*acc_ref_shared[1])/(self.g)/self.z_factor)
				if pitch_d > np.pi/6:
					pitch_d = np.pi/6
				elif pitch_d < -np.pi/6:
					pitch_d = -np.pi/6
				roll_d = self.roll_factor*np.arctan(-np.cos(pitch_d)*(a*acc_ref_shared[0]-b*acc_ref_shared[1])/(self.g)/self.z_factor)
				if roll_d > np.pi/6:
					roll_d = np.pi/6
				elif roll_d < -np.pi/6:
					roll_d = -np.pi/6
				thrust_d = (self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m*self.z_factor
			else:
				# print('loc_mode')
				pitch_d = att_ref_shared[1]
				roll_d = att_ref_shared[0]
				thrust_d = 0.05*(self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m
		else:
			pitch_d = np.arctan(-(b*acc_ref_shared[0]+a*acc_ref_shared[1])/(self.g-acc_ref_shared[2]))
			if pitch_d > np.pi/6:
				pitch_d = np.pi/6
			elif pitch_d < -np.pi/6:
				pitch_d = -np.pi/6
			roll_d = np.arctan(-np.cos(pitch_d)*(a*acc_ref_shared[0]-b*acc_ref_shared[1])/(self.g-acc_ref_shared[2]))
			if roll_d > np.pi/6:
				roll_d = np.pi/6
			elif roll_d < -np.pi/6:
				roll_d = -np.pi/6
			thrust_d = (self.g-acc_ref_shared[2])/np.cos(roll_d)/np.cos(pitch_d)*self.m
			# for small
			# if thrust_d > 1.5*self.m*self.g:
			# 	thrust_d = 1.5*self.m*self.g
			if thrust_d > 4.0*self.m*self.g:
				thrust_d = 4.0*self.m*self.g
		thrust_shared.value = thrust_d
		att_ref_shared[:] = [roll_d, pitch_d, yawd]


	def run(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, att_ref_shared, thrust_shared, reached, stop_shared, groundmode, mode, acc_ref_shared,state_fb,loc_mode,auto_mode):

		while 1:
			self.mode = mode.value
			att_fb = state_fb[6:9]
			# self.groundmode = groundmode.value
			if stop_shared.value == 1:
				break
			self.current_time = time.time()
			if self.current_time - self.last_loop_time > self.dt:
				self.acc_control(pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reached, acc_ref_shared,att_ref_shared,att_fb,auto_mode,groundmode)
				self.target_setup(acc_ref_shared,att_ref_shared,thrust_shared,loc_mode)
				self.last_loop_time = self.current_time
			else:
				time.sleep(0.001)

		self.stop()
	def stop(self):
		pass

class Master:
	def __init__(self):
		self.start_time = 0
		self.acc_ref_shared = multiprocessing.Array('f',3)
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
		self.stop_shared = multiprocessing.Value('i',0)
		self.thrust_shared = multiprocessing.Value('f',0)
		self.att_ref_shared = multiprocessing.Array('f',3)
		self.start = multiprocessing.Value('i',0)
		self.start.value = False
		self.mode = multiprocessing.Value('i',0)
		self.mode.value = 0
		self.loc_mode = multiprocessing.Value('i',0)
		self.loc_mode.value = False
		self.auto_mode = multiprocessing.Value('i',0)
		self.auto_mode.value = True
		self.keyboard = KeyboardControl(mode = 0,ground_mode=True)
		
		self.logger = CombinedLogger(folder_name='log_test_1113_new_modal')

		self.controller = Controller()
		### something went wrong with the state feedback shared, 0 yaw is always achieved, need to check the state feedback
		self.p_control = multiprocessing.Process(target=self.controller.run, 
												args=(self.pos_ref_shared, self.pos_fb_shared, self.vel_fb_shared,  self.vel_ref_shared, self.att_ref_shared, self.thrust_shared, self.reached, self.stop_shared, self.groundmode, self.mode,self.acc_ref_shared,self.state_fb_shared,self.loc_mode,self.auto_mode))
		self.cbframe = CombinedFrame()
		self.p_cbframe = multiprocessing.Process(target=self.cbframe.run, 
												args=(self.stop_shared, self.att_ref_shared, self.yaw_rate_shared, self.reached, self.start, self.state_fb_shared,self.thrust_shared,self.groundmode,self.loc_mode))

		self.op = Vicon()
		self.init_position = self.op.position
		self.init_pose = np.array([self.op.rpy[0],-self.op.rpy[1],-self.op.rpy[2]])
		self.rpy_vicon = np.array([self.op.rpy[0],-self.op.rpy[1],-self.op.rpy[2]])
		time.sleep(0.25)
		
	def run(self):
		self.start_time = time.time()
		self.last_loop_time = self.start_time
		print(self.start_time)
		loop_num = 0
		while 1:
			loop_num+=1
			# print('in master',self.start.value)
			if self.keyboard.stop == 1:
				self.stop_shared.value = 1
				break
			if loop_num < 100:
				self.init_position = self.op.position
			current_time = time.time()
			if current_time - self.last_loop_time > 0.005:
				self.pos_fb_shared[:] = self.op.position-self.init_position
				self.vel_fb_shared[:] = np.array([self.op.velocity[0],-self.op.velocity[1],-self.op.velocity[2]])# minus sign on y and z because of the coordinate system
				# self.pos_fb_shared[:] = [0.0,0.0,0.0]
				self.rpy_vicon = np.array([self.op.rpy[0],self.op.rpy[1],self.op.rpy[2]])
				self.keyboard.command.update()
				self.start.value = self.keyboard.start
				self.mode.value = self.keyboard.mode
				self.loc_mode.value = self.keyboard.loc_mode
				self.auto_mode.value = self.keyboard.auto_mode
				self.stop_shared.value = self.keyboard.stop
				self.pos_ref_shared[:] = self.keyboard.pos
				if not self.auto_mode.value:
					self.groundmode.value = self.keyboard.ground_mode
				if self.loc_mode.value:
					self.att_ref_shared[:] = self.keyboard.rpy #update all in local mode
				else:
					self.att_ref_shared[2] = self.keyboard.rpy[2] #update yaw
				# if self.controller.mode == 1:
				self.vel_ref_shared[:] = self.keyboard.vel
				self.yaw_rate_shared = self.keyboard.agv[2]
				# self.vel_fb_shared[:] = self.state_fb_shared[0:3]
				self.logger.log_append(int(round((current_time-self.start_time) * 1000)), int(round((current_time-self.last_loop_time) * 1000)),
									   self.pos_fb_shared[:], 
									   self.vel_fb_shared[:], self.state_fb_shared[6:9], self.state_fb_shared[3:6],
									   self.pos_ref_shared[:],[self.att_ref_shared[0],self.att_ref_shared[1],-self.att_ref_shared[2]],
									   self.vel_ref_shared[:], self.state_fb_shared[13:16],self.state_fb_shared[16:20],self.state_fb_shared[20:24],self.state_fb_shared[24:28],self.state_fb_shared[28:32],self.thrust_shared.value,self.rpy_vicon,self.acc_ref_shared[:],
									   self.state_fb_shared[10:13])
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

