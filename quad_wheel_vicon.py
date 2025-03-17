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

from crazyflie import SingleCF
# from crazyflie_imu import IMUCF
from new_logger import CombinedLogger
from gimbal_platform_keyboard import KeyboardControl
# from tcp_server_modify import Vicon
from tcp_server_modify_2 import Vicon
from test_write_txt import write_txt_file

logging.basicConfig(level=logging.ERROR)


class CombinedFrame:
	def __init__(self):
		self.qc_setup_time = 0.25
		self.update_rate = 0.005

		self.last_loop_time = time.time()
		self.current_time = self.last_loop_time
		self.log = []
		self.dt = 0


	def run(self, stop_shared,start,thrust_shared,torque_shared,state_fb_shared):

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
				self.qc.thrustd = thrust_shared.value
				self.qc.start = start.value
				self.qc.torque = torque_shared[:]
				self.qc._update_motors()
				self.dt = self.current_time - self.last_loop_time
				self.last_loop_time = self.current_time
				self.log.append(self.dt)

				state_fb_shared[:] = [self.qc.m1,self.qc.m2,self.qc.m3,self.qc.m4]
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
		self.att_ref_shared = np.array([0.0,0.0,0.0])
		self.reset = False
		self.groundmode = True
		self.g = 9.81
		self.z_factor = 0.7
		# self.m = 0.083 #for small
		self.m = 0.415 #for big
		self.roll_factor = 1.0
		self.pitch_factor = 1.0
		self.num = 0
		self.num_bad = 0 # TEST
		self.acc_body = np.array([0.0,0.0,0.0])
		self.Ixx = 6.3819E-05
		self.Iyy = 9.8489E-05
		self.Izz = 0.00015404
		self.Iyy_prime = 8.1398E-06
		self.Izz_prime = 0.1065**2*0.0801*2 + 1.3349E-05 + 0.0002545*2
		self.Ixx_frame = 2.0819E-05+0.00025432*2+0.1065**2*0.0801*2
		self.e_rpy = np.array([0.0,0.0,0.0])
	
	def acc_control(self, pos_ref_shared, pos_fb_shared, vel_fb_shared,  vel_ref_shared, acc_ref_shared,att_ref_shared,att_fb,auto_mode,groundmode):
		# acc_ref_shared[:] = self.pKd*(np.array(vel_ref_shared) - np.array(vel_fb_shared))
		if self.reset:
			self.reset = False
		if auto_mode.value:
			if pos_fb_shared[2] > 0.05 or pos_ref_shared[2] > 0.05:
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
			if self.groundmode != groundmode.value:
				self.reset = True
				self.groundmode = groundmode.value
			else:
				if self.groundmode:
					print('groundmode')
				else:
					print('airmode')
		
		if self.groundmode:
			self.pKp = np.array([14.0,14.0,0])
			self.pKi = np.array([0.5,0.5,0])
			self.pKd = np.array([5.5,5.5,0])
			self.rKp = np.array([300,250,20])
			self.rKi = np.array([15,15,0])
			self.rKd = np.array([90,40,5])
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
			self.pKp = np.array([14.0,14.0,80.0])
			self.pKi = np.array([1.5,1.5,60.0])
			self.pKd = np.array([8.0,8.0,30.0])
			self.rKp = np.array([300,250,20])
			self.rKi = np.array([15,15,0])
			self.rKd = np.array([90,40,5])
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
			### for large
			if np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2) < 0.001 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.05:
				# reset.value = True
				self.z_factor = 0.3
				self.roll_factor = 1.0
			elif np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2) < 1.5 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.2:
				# reset.value = False
				x = np.max([(np.sqrt(0.5*self.acc_body[0]**2+1.5*self.acc_body[1]**2)-0.001)/1.499,(np.abs(att_ref_shared[2]+att_fb[2])-0.05)/0.15])
				self.z_factor = 0.3 + 0.5*x
				# self.z_factor = 0.25 + 0.55*np.log2(1+x) ##seems to be good
				# self.z_factor = 0.15 - 0.65 * x * (x-2)##seems to be the best
				# self.z_factor = 0.05 + 0.75 * x ** (1/2.0)
				self.roll_factor = 1.0# for linear
				# self.roll_factor = 1.3# for log
			else:
				# reset.value = False
				self.z_factor = 0.8
				self.roll_factor = 1.0# for linear
				# self.roll_factor = 1.3# for log


			# ### for small
			# if np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2) < 0.001 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.05:
			# 	# reset.value = True
			# 	self.z_factor = 0.4
			# elif np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2) < 1.5 and np.abs(att_ref_shared[2]+att_fb[2]) < 0.2:
			# 	# reset.value = False
			# 	x = np.max([(np.sqrt(acc_ref_shared[0]**2+acc_ref_shared[1]**2)-0.001)/1.499,(np.abs(att_ref_shared[2]+att_fb[2])-0.05)/0.15])
			# 	# self.z_factor = 0.4 + 0.5*x
			# 	# self.z_factor = 0.4 + 0.5*np.log2(1+x) ##seems to be good
			# 	self.z_factor = 0.4 - 0.5 * x * (x-2)##seems to be the best
			# 	self.roll_factor = 1.0# for linear
			# 	# self.roll_factor = 1.3# for log
			# else:
			# 	# reset.value = False
			# 	self.z_factor = 0.9
			# 	self.roll_factor = 1.0# for linear
			# 	# self.roll_factor = 1.3# for log

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
		self.att_ref_shared = np.array([roll_d, pitch_d, yawd])
		self.att_ref_shared = np.array([0.0,0.0,0.0])
		# att_ref_shared[:] = [roll_d, pitch_d, yawd]


	def torque_setup(self,torque_shared,att_fb_shared,omega_fb_shared,rpyPID_shared):
		angle_dif = self.frame_roll - att_fb_shared[0]
		# print(angle_dif)
		I_new = np.array([[self.Ixx,0,0],
                        [0,self.Iyy +self.Iyy_prime*np.cos(angle_dif)**2+ self.Izz_prime*np.sin(angle_dif)**2,(self.Izz_prime-self.Iyy_prime)*np.sin(angle_dif)*np.cos(angle_dif)],
                        [0,(self.Izz_prime-self.Iyy_prime)*np.sin(angle_dif)*np.cos(angle_dif),self.Izz +self.Izz_prime*np.cos(angle_dif)**2+ self.Iyy_prime*np.sin(angle_dif)**2]])
		rpy_err = self.att_ref_shared - att_fb_shared[:]
		if self.reset:
			self.e_rpy = np.array([0.0,0.0,0.0])
		self.e_rpy = self.e_rpy + rpy_err*self.dt
		omega_dot = np.array([self.rKi[0]*self.e_rpy[0]-self.rKd[0]*omega_fb_shared[0]+self.rKp[0]*rpy_err[0],
                              self.rKi[1]*self.e_rpy[1]-self.rKd[1]*omega_fb_shared[1]+self.rKp[1]*rpy_err[1],
                              self.rKi[2]*self.e_rpy[2]-self.rKd[2]*(omega_fb_shared[2] - 0.0)+self.rKp[2]*rpy_err[2]])# 0.0 is the yaw rate ref
		rpyPID_shared[:] = omega_dot
		Iw = np.array([I_new[0][0]*omega_fb_shared[0],
                       I_new[1][1]*omega_fb_shared[1]+I_new[1][2]*omega_fb_shared[2],
                       I_new[2][2]*omega_fb_shared[2]+I_new[2][1]*omega_fb_shared[1]])
		Iw_dot = np.array([I_new[0][0]*omega_dot[0],
                            I_new[1][1]*omega_dot[1]+I_new[1][2]*omega_dot[2],
                            I_new[2][2]*omega_dot[2]+I_new[2][1]*omega_dot[1]])
		tau = Iw_dot + np.array([omega_fb_shared[1]*Iw[2]-omega_fb_shared[2]*Iw[1],
                                 omega_fb_shared[2]*Iw[0]-omega_fb_shared[0]*Iw[2],
                                 omega_fb_shared[0]*Iw[1]-omega_fb_shared[1]*Iw[0]])
		tau[0] -= 0.21*angle_dif
		if np.abs(tau[0]) > 0.1:
			tau[0]= 0.1*np.sign(tau[0])
		if np.abs(tau[1]) > 0.1:
			tau[1]= 0.1*np.sign(tau[1])
		if np.abs(tau[2]) > 0.05:
			tau[2]= 0.05*np.sign(tau[2])

		torque_shared[:] = tau
		
		



	def run(self, pos_ref_shared, pos_fb_shared, vel_ref_shared, vel_fb_shared,
		  att_ref_shared, att_fb_shared, omega_fb_shared, thrust_shared, frame_roll,
		    ground_mode,velocity_mode,loco_mode,auto_mode,torque_shared,stop_shared,rpyPID_shared,
			acc_ref_shared,att_desired_shared):
		while 1:
			self.mode = velocity_mode.value
			self.frame_roll = frame_roll.value
			self.current_time = time.time()
			if stop_shared.value == 1:
				break
			if self.current_time - self.last_loop_time > self.dt:
				self.acc_control(pos_ref_shared, pos_fb_shared, vel_fb_shared, vel_ref_shared, acc_ref_shared,att_ref_shared,att_fb_shared,auto_mode,ground_mode)
				self.target_setup(acc_ref_shared,att_ref_shared,thrust_shared,loco_mode)
				self.torque_setup(torque_shared,att_fb_shared,omega_fb_shared,rpyPID_shared)
				self.last_loop_time = self.current_time
				att_desired_shared[:] = self.att_ref_shared
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
		self.state_fb_shared = multiprocessing.Array('f',4)
		self.groundmode = multiprocessing.Value('i',0)
		self.groundmode.value = True
		self.pos_ref_shared = multiprocessing.Array('f',3)
		self.pos_fb_shared = multiprocessing.Array('f',3)
		self.vel_fb_shared = multiprocessing.Array('f',3)
		self.omega_fb_shared = multiprocessing.Array('f',3)
		self.stop_shared = multiprocessing.Value('i',0)
		self.thrust_shared = multiprocessing.Value('f',0)
		self.att_ref_shared = multiprocessing.Array('f',3)
		self.start = multiprocessing.Value('i',0)
		self.start.value = False
		self.mode = multiprocessing.Value('i',0)
		self.mode.value = False
		self.loc_mode = multiprocessing.Value('i',0)
		self.loc_mode.value = False
		self.auto_mode = multiprocessing.Value('i',0)
		self.auto_mode.value = True
		self.keyboard = KeyboardControl(mode = 0,ground_mode=True)
		self.frame_roll = multiprocessing.Value('f',0)
		self.att_fb_shared = multiprocessing.Array('f',3)
		self.torque_shared = multiprocessing.Array('f',3)
		self.rpyPID_shared = multiprocessing.Array('f',3)
		self.rpyPID_shared[:] = [0.0,0.0,0.0]
		self.att_desired_shared = multiprocessing.Array('f',3)
		
		self.logger = CombinedLogger(folder_name='log_test_0306_new_model')

		self.controller = Controller()
		### something went wrong with the state feedback shared, 0 yaw is always achieved, need to check the state feedback
		self.p_control = multiprocessing.Process(target=self.controller.run, 
												args=(self.pos_ref_shared, self.pos_fb_shared, self.vel_ref_shared,self.vel_fb_shared,
				  self.att_ref_shared, self.att_fb_shared, self.omega_fb_shared, self.thrust_shared, self.frame_roll,
				  self.groundmode,self.mode,self.loc_mode,self.auto_mode,self.torque_shared,self.stop_shared,self.rpyPID_shared,self.acc_ref_shared,self.att_desired_shared))
		self.cbframe = CombinedFrame()
		self.p_cbframe = multiprocessing.Process(target=self.cbframe.run, 
												args=(self.stop_shared,self.start,self.thrust_shared,self.torque_shared,self.state_fb_shared))

		self.op = Vicon()
		self.init_position = self.op.body_position
		self.quat = self.op.body_rotation	
		self.init_rpy = np.array([0.0,0.0,0.0])
		self.traj_mode = False
		self.traj_ref = np.array([[0.0,0.0,0.0]])
		self.traj_fb = np.array([[0.0,0.0,0.0]])
		self.power = 0
		time.sleep(0.25)
	
	def quat2rpy(self,quat):
		roll = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))
		pitch = np.arcsin(2*(quat[0]*quat[2]-quat[3]*quat[1]))
		yaw = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))
		return np.array([roll,pitch,yaw])

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
				self.init_position = self.op.body_position
				self.init_rpy = self.quat2rpy(self.op.body_rotation)

			current_time = time.time()
			if current_time - self.last_loop_time > 0.01:
				self.pos_fb_shared[:] = self.op.body_position-self.init_position
				self.vel_fb_shared[:] = np.array([self.op.body_velocity[0],self.op.body_velocity[1],self.op.body_velocity[2]])# minus sign on y and z because of the coordinate system
				self.att_fb_shared[:] = self.quat2rpy(self.op.body_rotation)
				self.omega_fb_shared[:] = np.array([self.op.body_rotation_rate[0],self.op.body_rotation_rate[1],self.op.body_rotation_rate[2]])
				self.frame_roll.value = self.op.frame_rotation[0]
				# self.pos_fb_shared[:] = [0.0,0.0,0.0]
				# self.rpy_vicon = np.array([self.op.rpy[0],self.op.rpy[1],self.op.rpy[2]])
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
									   self.pos_fb_shared[:],self.pos_ref_shared[:],self.vel_fb_shared[:],self.vel_ref_shared[:],
									   self.att_fb_shared[:],self.att_ref_shared[:],self.rpyPID_shared[:],self.omega_fb_shared[:],
									   self.torque_shared[:],self.state_fb_shared[:],self.att_desired_shared[:])
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

