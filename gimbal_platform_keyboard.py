"""
Use keyboard to configure trajectory
mode:
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
	def __init__(self, mode=0, ground_mode=True):
		self.command = tk.Tk()
		self.command.bind_all("<Key>", self.key_input)
		self.mode = mode

		self.pos = [0,0,0.0] # x y z
		self.vel = [0.0,0.0,0.0] # velocity
		self.rpy = [0.0,0.0,0] # roll pitch yaw
		#self.rpy = [0.5, -0.6, 1]  # roll pitch yaw
		self.agv = [0.0,0.0,0.0] # angular velocity
		self.start = True
		self.angle_step = 0.1
		self.pos_step = 0.05
		self.z_step = 0.1

		self.stop = 0
		self.landing_overwrite = 0
		self.ground_mode = ground_mode

	def key_input(self, event):
		key_press = event.keysym.lower()
		print(key_press)

		if self.mode==0:
			if key_press == 'w':
				self.pos[0] -= self.pos_step
			elif key_press == 's':
				self.pos[0] += self.pos_step
			elif key_press == 'a':
				self.pos[1] -= self.pos_step
			elif key_press == 'd':
				self.pos[1] += self.pos_step

			elif key_press == 'i':
				self.pos[0] -= 0.1*self.pos_step
			elif key_press == 'k':
				self.pos[0] += 0.1*self.pos_step
			elif key_press == 'j':
				self.pos[1] -= 0.1*self.pos_step
			elif key_press == 'l':
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
		elif self.mode==1:
			if key_press == 'w':
				self.vel[0] -= self.pos_step
			elif key_press == 's':
				self.vel[0] += self.pos_step
			elif key_press == 'a':
				self.vel[1] -= self.pos_step
			elif key_press == 'd':
				self.vel[1] += self.pos_step

			elif key_press == 'i':
				self.vel[0] -= 0.1*self.pos_step
			elif key_press == 'k':
				self.vel[0] += 0.1*self.pos_step
			elif key_press == 'j':
				self.vel[1] -= 0.1*self.pos_step
			elif key_press == 'l':
				self.vel[1] += 0.1*self.pos_step

			elif key_press == 'q':
				self.agv[2] += self.angle_step
			elif key_press == 'e':
				self.agv[2] -= self.angle_step
			elif key_press == 'control_l':
				self.vel[2] -= self.z_step
			elif key_press == 'alt_l':
				self.vel[2] += self.z_step
			elif key_press == 'control_r':
				self.vel[2] -= 0.1*self.z_step
			elif key_press == 'alt_r':
				self.vel[2] += 0.1*self.z_step
		if key_press == 'space':
			if self.mode == 0:
				self.mode = 1
				print("change mode to velocity")
			else:
				self.mode = 0
				print("change mode to position")
		if key_press == 't':
			if self.ground_mode:
				print("change mode to aerial mode")
				self.ground_mode = False
			else:
				print("change mode to ground mode")
				self.ground_mode = True
		if key_press == 'escape':
			self.stop = 1
			self.start = False
			print("x=%s y=%s z=%s" % (self.pos[0], self.pos[1], self.pos[2]))



if __name__ == '__main__':

	remoter = KeyboardControl(mode=0)
	while remoter.stop == False:
		remoter.command.update()
		time.sleep(0.01)