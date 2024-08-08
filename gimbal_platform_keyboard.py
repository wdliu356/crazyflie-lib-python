"""
Use keyboard to configure trajectory
control_mode:
	'attitude': manual control of platform attitude
				and designed trajectories
	'position': manual control of platform position

ILC trajectory: refer to ILC_learning_process.py
"""
import math
import tkinter as tk
import time
import os
import sys
import csv
import threading
import multiprocessing
import numpy as np
# import transformations as tf

class KeyboardControl():
	def __init__(self, control_mode='attitude'):
		self.command = tk.Tk()
		self.command.bind_all("<Key>", self.key_input)
		self.control_mode = control_mode

		self.pos = [0,0,0.0] # x y z
		self.vel = [0.0,0.0,0.0] # velocity
		self.rpy = [0.0,0.0,0] # roll pitch yaw
		#self.rpy = [0.5, -0.6, 1]  # roll pitch yaw
		self.agv = [0.0,0.0,0.0] # angular velocity

		self.angle_step = 0.1
		self.pos_step = 0.05
		self.z_step = 0.1

		self.stop = 0
		self.landing_overwrite = 0

	def key_input(self, event):
		key_press = event.keysym.lower()
		print(key_press)

		if self.control_mode=='attitude':
			if key_press == 'w':
				# self.rpy[0] -= self.angle_step
				self.pos[0] -= self.pos_step
			elif key_press == 's':
				# self.rpy[0] += self.angle_step
				self.pos[0] += self.pos_step
			elif key_press == 'a':
				# self.rpy[1] -= self.angle_step
				self.pos[1] -= self.pos_step
			elif key_press == 'd':
				# self.rpy[1] += self.angle_step
				self.pos[1] += self.pos_step
			elif key_press == 't':
				# self.rpy[0] -= self.angle_step
				self.pos[0] -= 0.1*self.pos_step
			elif key_press == 'g':
				# self.rpy[0] += self.angle_step
				self.pos[0] += 0.1*self.pos_step
			elif key_press == 'f':
				# self.rpy[1] -= self.angle_step
				self.pos[1] -= 0.1*self.pos_step
			elif key_press == 'h':
				# self.rpy[1] += self.angle_step
				self.pos[1] += 0.1*self.pos_step
			elif key_press == 'q':
				self.rpy[2] += self.angle_step
			elif key_press == 'e':
				self.rpy[2] -= self.angle_step
			elif key_press == 'control_l':
				self.pos[2] = self.pos[2]-self.z_step
			elif key_press == 'alt_l':
				self.pos[2] = self.pos[2]+self.z_step
			elif key_press == 'control_r':
				self.pos[2] = self.pos[2]-0.1*self.z_step
			elif key_press == 'alt_r':
				self.pos[2] = self.pos[2]+0.1*self.z_step
			# elif key_press == 't':
			# 	print('trajectory combined')
			# 	run_t = threading.Thread(target=self.traj_combined, name='trajt')
			# 	run_t.start()
			# elif key_press == 'y':
			# 	print('trajectory pitch 90')
			# 	run_y = threading.Thread(target=self.traj_pitch90, name='trajy')
			# 	run_y.start()
			# elif key_press == 'u':
			# 	print('trajectory pitch ramp 90')
			# 	run_u = threading.Thread(target=self.traj_ramp90, name='traju')
			# 	run_u.start()
			# elif key_press == 'i':
			# 	print('trajectory cross cos ramp 90/180')
			# 	run_i = threading.Thread(target=self.traj_cross, name='traji')
			# 	run_i.start()
			# elif key_press == 'o':
			# 	print('trajectory pitch 360')
			# 	run_o = threading.Thread(target=self.traj_360, name='trajo')
			# 	run_o.start()
			# elif key_press == 'l':
			# 	print('landing')
			# 	run_l = threading.Thread(target=self.traj_landing, name='trajl')
			# 	run_l.start()
			# elif key_press == 'p':
			# 	print('ref trajectory of ILC')
			# 	run_p = threading.Thread(target=self.traj_ILC, name='trajp')
			# 	run_p.start()
			# elif key_press == 'z':
			# 	print('ref trajectory of aerial-vkc1')
			# 	run_v = threading.Thread(target=self.traj_aerialvkc0, name='trvkcx')
			# 	run_v.start()
			elif key_press == 'x':
				print('ref trajectory of aerial-vkc1')
				run_v = threading.Thread(target=self.traj_aerialvkc1, name='trvkcx')
				run_v.start()
			elif key_press == 'c':
				print('ref trajectory of aerial-vkc2')
				run_v = threading.Thread(target=self.traj_aerialvkc2, name='trvkcc')
				run_v.start()
			elif key_press == 'v':
				print('ref trajectory of aerial-vkc3')
				run_v = threading.Thread(target=self.traj_aerialvkc3, name='trvkcv')
				run_v.start()
			# elif key_press == 'b':
			# 	print('ref trajectory of aerial-vkc4')
			# 	run_v = threading.Thread(target=self.traj_aerialvkc4, name='trvkcb')
			# 	run_v.start()
			# elif key_press == 'n':
			# 	print('ref trajectory of aerial-vkc5')
			# 	run_v = threading.Thread(target=self.traj_aerialvkc5, name='trvkcn')
			# 	run_v.start()
			# elif key_press == 'm':
			# 	print('ref trajectory of aerial-vkc6')
			# 	run_v = threading.Thread(target=self.traj_aerialvkc6, name='trvkcsm')
			# 	run_v.start()
			# # elif key_press == 'f':
			# 	print('ref trajectory of dowan_wash_four')
			# 	run_f = threading.Thread(target=self.traj_downwash_four, name='trajf')
			# 	run_f.start()
			# elif key_press == 'k':
			# 	print('small kick to initiate ILCFF')
			# 	self.pos[0] = 0.001
			elif key_press == 'd':
				self.pos[1] += self.pos_step
			elif key_press == 'a':
				self.pos[1] -= self.pos_step
			elif key_press == 's':
				self.pos[0] -= self.pos_step
			elif key_press == 'w':
				self.pos[0] += self.pos_step
			elif key_press == 'control_l':
				self.pos[2] = self.pos[2]-self.z_step
			elif key_press == 'alt_l':
				self.pos[2] = self.pos[2]+self.z_step
			elif key_press == 'escape':
				self.stop = 1
			print("x=%s y=%s z=%s" % (self.pos[0], self.pos[1], self.pos[2]))



if __name__ == '__main__':

	remoter = KeyboardControl(control_mode='attitude')
	while remoter.stop == False:
		remoter.command.update()
		time.sleep(0.01)