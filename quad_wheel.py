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


	def run(self, stop_shared, att_ref_shared, yaw_rate_shared,reset,start,state_fb_shared,thrust_shared,groundmode,loc_mode,frame_roll,yaw_fb):

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
				self.qc.frame_roll = frame_roll.value
				# print('in cbframe',self.qc.start.value)
				self.qc.reset = reset.value
				self.qc.loc_mode = loc_mode.value
				self.qc.yawfb = yaw_fb.value
				self.qc._update_motors()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				self.log.append(self.dt)

				state_fb_shared[:] = [self.qc.vxfb, self.qc.vyfb, self.qc.vzfb, self.qc.rollratefb, self.qc.pitchratefb, self.qc.yawratefb, self.qc.roll, self.qc.pitch, self.qc.yawfb, self.qc.thrust,self.qc.rollreceived,
						  self.qc.pitchreceived,self.qc.yawreceived,self.qc.rolltorque,self.qc.pitchtorque,self.qc.yawtorque,self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4,self.qc.u1,self.qc.u2,self.qc.u3,self.qc.u4,
						  self.qc.m1req,self.qc.m2req,self.qc.m3req,self.qc.m4req,self.qc.thrust1,self.qc.thrust2,self.qc.thrust3,self.qc.thrust4,self.qc.frame_roll]
			else:
				time.sleep(0.001)

		self.stop()

	def stop(self):
		self.qc._stop_crazyflie()


###### Important note
# Position is in vicon frame, velocity is in original body frame
# The difference is the direction of the y axis and z axis
# The y axis of the original body frame is pointing to the right, the z axis is pointing down
# The y axis of the vicon frame is pointing to the left, the z axis is pointing up
class Controller:
	def __init__(self):
		# self.qc_setup_time = 0.25
		self.dt = 0.01
		self.mode = 0 # 0 for position, 1 for velocity
		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.perr = np.array([0.0,0.0,0.0])
		self.verr = np.array([0.0,0.0,0.0])
		self.pKp = np.array([5.0,5.0,15.0])# minus sign on y and z because of the coordinate system
		self.pKi = np.array([0.1,0.1,2.5])# minus sign on y and z because of the coordinate system
		self.pKd = np.array([3.5,3.5,3.0])
		self.reset = False
		self.groundmode = True
		self.g = 9.81
		self.z_factor = 0.7
		self.m = 0.083 #for small
		# self.m = 0.415 #for big
		self.roll_factor = 1.0
		self.pitch_factor = 1.0
		self.num = 0
		self.num_bad = 0 # TEST
		self.acc_body = np.array([0.0,0.0,0.0])
	
	def acc_control(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reset, acc_ref_shared,att_ref_shared,att_fb,auto_mode,groundmode):
		# acc_ref_shared[:] = self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
		if reset.value:
			reset.value = False
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
				reset.value = True
			else:
				print('airmode')
				reset.value = True
		
		if self.groundmode:
			# self.pKp = np.array([14.0,14.0,0])
			# self.pKi = np.array([0.5,0.5,0])
			# self.pKd = np.array([5.5,5.5,0])
			# self.pKp = np.array([7.0,-7.0,-0.0])
			# self.pKi = np.array([1.2,-1.2,-0.0])
			# self.pKd = np.array([5.0,5.0,0.0])
			self.pKp = np.array([6.5,6.5,15.0])
			self.pKi = np.array([0.1,0.1,2.5])
			self.pKd = np.array([4.5,4.5,3.0])
		else:
			# for small
			self.pKp = np.array([6.5,6.5,15.0])
			self.pKi = np.array([0.35,0.35,5.5])
			self.pKd = np.array([5.0,5.0,10.0])

			# # for big
			# self.pKp = np.array([14.0,14.0,80.0])
			# self.pKi = np.array([1.5,1.5,60.0])
			# self.pKd = np.array([8.0,8.0,30.0])
		self.perr += (np.array(pos_ref_shared) - np.array(pos_fb_shared)) * self.dt
		for i in range(3):
			if abs(self.perr[i]) > 10:
				self.perr[i] = 10*np.sign(self.perr[i])
		if self.mode == 0:# position mode
			acc_ref_shared[:] = self.pKp * (np.array(pos_ref_shared) - np.array(pos_fb_shared)) + self.pKi * self.perr + self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
		else:
			acc_ref_shared[:] = self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
		self.acc_body[0] = np.cos(att_fb[2])*acc_ref_shared[0] + np.sin(att_fb[2])*acc_ref_shared[1]
		self.acc_body[1] = -np.sin(att_fb[2])*acc_ref_shared[0] + np.cos(att_fb[2])*acc_ref_shared[1]
		if self.groundmode:
			# ### for large
			# if np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2) < 0.001 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.05:
			# 	# reset.value = True
			# 	self.z_factor = 0.05
			# 	self.roll_factor = 1.0
			# elif np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2) < 1.5 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.2:
			# 	# reset.value = False
			# 	x = np.max([(np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2)-0.001)/1.499,(np.abs(att_ref_shared[2]+att_fb[2])-0.05)/0.15])
			# 	self.z_factor = 0.5 + 0.3*x
			# 	# self.z_factor = 0.25 + 0.55*np.log2(1+x) ##seems to be good
			# 	# self.z_factor = 0.15 - 0.65 * x * (x-2)##seems to be the best
			# 	# self.z_factor = 0.05 + 0.75 * x ** (1/2.0)
			# 	self.roll_factor = 1.0# for linear
			# 	# self.roll_factor = 1.3# for log
			# else:
			# 	# reset.value = False
			# 	self.z_factor = 0.8
			# 	self.roll_factor = 1.0# for linear
			# 	# self.roll_factor = 1.3# for log


			### for small
			if np.sqrt(0.2*self.acc_body[0]**2+2.5*self.acc_body[1]**2) < 0.001 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.05:
				# reset.value = True
				self.z_factor = 0.4
			elif np.sqrt(0.2*self.acc_body[0]**2+2.5*self.acc_body[1]**2) < 1.5 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.2:
				# reset.value = False
				x = np.max([(np.sqrt(0.2*self.acc_body[0]**2+2.5*self.acc_body[1]**2)-0.001)/1.499,(np.abs(att_ref_shared[2]+att_fb[2])-0.05)/0.15])
				# self.z_factor = 0.4 + 0.5*x
				# self.z_factor = 0.4 + 0.5*np.log2(1+x) ##seems to be good
				self.z_factor = 0.4 - 0.6 * x * (x-2)##seems to be the best
				# self.roll_factor = 1.0# for linear
				self.roll_factor = 2.0# for log
			else:
				# reset.value = False
				self.z_factor = 1.0
				# self.roll_factor = 1.0# for linear
				self.roll_factor = 2.0# for log

			# print('acc_diff,yaw_diff,z_factor',[np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2),np.abs(att_ref_shared[2]-att_fb[2]),self.z_factor])
			# print('yaw_fb,yaw_ref',[att_fb[2],att_ref_shared[2]])
					
		

	def target_setup(self,acc_ref_shared,att_ref_shared,thrust_shared,loc_mode):
		yawd = att_ref_shared[2]
		a = np.sin(yawd)
		b = np.cos(yawd)
		if self.groundmode:
			if not loc_mode.value:
				pitch_d = self.pitch_factor*np.arctan((b*acc_ref_shared[0]+a*acc_ref_shared[1])/(self.g)/self.z_factor)
				if pitch_d > np.pi/6:
					pitch_d = np.pi/6
				elif pitch_d < -np.pi/6:
					pitch_d = -np.pi/6
				roll_d = self.roll_factor*np.arctan(np.cos(pitch_d)*(a*acc_ref_shared[0]-b*acc_ref_shared[1])/(self.g)/self.z_factor)
				if roll_d > np.pi/6:
					roll_d = np.pi/6
				elif roll_d < -np.pi/6:
					roll_d = -np.pi/6
				thrust_d = (self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m*self.z_factor
				# thrust_d = (self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m*0.5
				
			else:
				# print('loc_mode')
				pitch_d = att_ref_shared[1]
				roll_d = att_ref_shared[0]
				thrust_d = 0.025*(self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m
		else:
			pitch_d = np.arctan((b*acc_ref_shared[0]+a*acc_ref_shared[1])/(self.g+acc_ref_shared[2]))
			if pitch_d > np.pi/6:
				pitch_d = np.pi/6
			elif pitch_d < -np.pi/6:
				pitch_d = -np.pi/6
			roll_d = np.arctan(np.cos(pitch_d)*(a*acc_ref_shared[0]-b*acc_ref_shared[1])/(self.g+acc_ref_shared[2]))
			roll_d = roll_d - 0.3*self.frame_roll## for frame stability
			if roll_d > np.pi/6:
				roll_d = np.pi/6
			elif roll_d < -np.pi/6:
				roll_d = -np.pi/6
			thrust_d = (self.g+acc_ref_shared[2])/np.cos(roll_d)/np.cos(pitch_d)*self.m
			# for small
			# if thrust_d > 1.5*self.m*self.g:
			# 	thrust_d = 1.5*self.m*self.g
			if thrust_d > 4.0*self.m*self.g:
				thrust_d = 4.0*self.m*self.g
		# thrust_d = 0.05*(self.g)/np.cos(roll_d)/np.cos(pitch_d)*self.m
		thrust_shared.value = thrust_d
		att_ref_shared[:] = [roll_d, pitch_d, yawd]
		# att_ref_shared[: ]= [0.0, 0.0, 0.0]


	def run(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, att_ref_shared, thrust_shared, reset, stop_shared, groundmode, mode, acc_ref_shared,state_fb,loc_mode,auto_mode,frame_roll):

		while 1:
			self.mode = mode.value
			self.frame_roll = frame_roll.value
			att_fb = state_fb[6:9]
			# self.groundmode = groundmode.value
			if stop_shared.value == 1:
				break
			self.current_time = time.time()
			if self.current_time - self.last_loop_time > self.dt:
				self.acc_control(pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, reset, acc_ref_shared,att_ref_shared,att_fb,auto_mode,groundmode)
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
		self.state_fb_shared = multiprocessing.Array('f',33)
		self.reset = multiprocessing.Value('i',0)
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
		self.frame_roll = multiprocessing.Value('f',0)
		self.yaw_fb = multiprocessing.Value('f',0)
		
		self.logger = CombinedLogger(folder_name='log_test_0307_small')

		self.controller = Controller()
		### something went wrong with the state feedback shared, 0 yaw is always achieved, need to check the state feedback
		self.p_control = multiprocessing.Process(target=self.controller.run, 
												args=(self.pos_ref_shared, self.pos_fb_shared, self.vel_fb_shared,  self.vel_ref_shared, self.att_ref_shared, self.thrust_shared, self.reset, self.stop_shared, self.groundmode, self.mode,self.acc_ref_shared,self.state_fb_shared,self.loc_mode,self.auto_mode,self.frame_roll))
		self.cbframe = CombinedFrame()
		self.p_cbframe = multiprocessing.Process(target=self.cbframe.run, 
												args=(self.stop_shared, self.att_ref_shared, self.yaw_rate_shared, self.reset, self.start, self.state_fb_shared,self.thrust_shared,self.groundmode,self.loc_mode,self.frame_roll,self.yaw_fb))

		self.op = Vicon()
		self.init_position = self.op.frame_position
		self.rpy_vicon = np.array([0.0,0.0,0.0])
		self.traj_mode = False
		self.traj_ref = np.array([[0.0,0.0,0.0]])
		self.traj_fb = np.array([[0.0,0.0,0.0]])
		self.power = 0
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
				self.start.value = False
				break
			if loop_num < 100:
				self.init_position = self.op.frame_position
			current_time = time.time()
			if current_time - self.last_loop_time > 0.01:
				self.pos_fb_shared[:] = self.op.frame_position-self.init_position
				self.vel_fb_shared[:] = np.array([self.op.frame_velocity[0],self.op.frame_velocity[1],self.op.frame_velocity[2]])# minus sign on y and z because of the coordinate system
				quat = self.op.body_rotation
				### The quat is in the order of w,x,y,z
				### Transfer the quat to roll pitch yaw in zyx order

				roll = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))
				pitch = np.arcsin(2*(quat[0]*quat[2]-quat[3]*quat[1]))
				yaw = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))
				self.rpy_vicon = np.array([roll,pitch,yaw])

				quat_frame = self.op.frame_rotation
				self.frame_roll.value = np.arctan2(2*(quat_frame[0]*quat_frame[1]+quat_frame[2]*quat_frame[3]),1-2*(quat_frame[1]**2+quat_frame[2]**2))
				self.frame_yaw = np.arctan2(2*(quat_frame[0]*quat_frame[3]+quat_frame[1]*quat_frame[2]),1-2*(quat_frame[2]**2+quat_frame[3]**2))
				# self.pos_fb_shared[:] = [0.0,0.0,0.0]
				# self.rpy_vicon = np.array([self.op.rpy[0],self.op.rpy[1],self.op.rpy[2]])
				# self.frame_roll.value = self.rpy_vicon[0]
				self.yaw_fb.value = self.frame_yaw
				self.keyboard.command.update()
				self.traj_mode = self.keyboard.traj_mode
				self.start.value = self.keyboard.start
				self.mode.value = self.keyboard.mode
				self.loc_mode.value = self.keyboard.loc_mode
				self.auto_mode.value = self.keyboard.auto_mode
				self.stop_shared.value = self.keyboard.stop
				self.pos_ref_shared[:] = self.keyboard.pos
				if self.traj_mode:
					self.traj_ref = np.vstack((self.traj_ref,self.keyboard.pos))
					self.traj_fb = np.vstack((self.traj_fb,self.pos_fb_shared[:]))
					self.power += self.thrust_shared.value*0.01

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
									   self.pos_ref_shared[:],[self.att_ref_shared[0],self.att_ref_shared[1],self.att_ref_shared[2]],
									   self.vel_ref_shared[:], self.state_fb_shared[13:16],self.state_fb_shared[16:20],self.state_fb_shared[20:24],self.state_fb_shared[24:28],self.state_fb_shared[28:32],self.thrust_shared.value,self.rpy_vicon,self.acc_ref_shared[:],
									   self.state_fb_shared[10:13])
				self.last_loop_time = current_time
			else:
				time.sleep(0.0001)
		
		rmse = np.sqrt(np.mean(np.linalg.norm(self.traj_ref[:,0:2]-self.traj_fb[:,0:2],axis=1)**2))
		print('rmse',rmse)
		print('power',self.power)

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

