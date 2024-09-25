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
		self.wp_index = 0
		self.start = True
		self.angle_step = 0.1
		self.pos_step = 0.05
		self.z_step = 0.1
		self.auto_mode = True
		self.stop = 0
		self.landing_overwrite = 0
		self.ground_mode = ground_mode
		self.loc_mode = False

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
				if self.rpy[2] > np.pi:
					self.rpy[2] -= 2*np.pi
			elif key_press == 'e':
				self.rpy[2] -= self.angle_step
				if self.rpy[2] < -np.pi:
					self.rpy[2] += 2*np.pi
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
		if key_press == 'z':### along the trajectory
			for i in range(10000):
				x = 2*np.pi/10000*i
				self.pos[0] = np.sin(2*x)
				self.pos[1] = 2*(1-np.cos(x))
				self.pos[2] = 0
				self.rpy[2] = np.arctan2(2*(1-np.cos(2*np.pi/10000*(i+1)))-2*(1-np.cos(x)),np.sin(2*2*np.pi/10000*(i+1))-np.sin(2*x))
				if key_press == 'escape':
					break
				time.sleep(0.005)

		elif key_press == 'x':### constant yaw rate
			for i in range(10000):
				x = 2*np.pi/10000*i
				self.pos[0] = np.sin(2*x)
				self.pos[1] = 2*(1-np.cos(x))
				self.pos[2] = 0
				self.agv[2] = 0.2
				if key_press == 'escape':
					break
				time.sleep(0.005)

		elif key_press == 'c':### constant yaw angle
			for i in range(10000):
				x = 2*np.pi/10000*i
				self.pos[0] = np.sin(2*x)
				self.pos[1] = 2*(1-np.cos(x))
				self.pos[2] = 0
				self.rpy[2] = 0
				if key_press == 'escape':
					break
				time.sleep(0.005)

		elif key_press == 'v':### changing alttitude
			for i in range(10000):
				x = 2*np.pi/10000*i
				self.pos[0] = np.sin(2*x)
				self.pos[1] = 2*(1-np.cos(x))
				self.pos[2] = 1/(0.3*np.sqrt(2*np.pi))*np.exp(-x**2/2/0.6**2)+1/(0.3*np.sqrt(2*np.pi))*np.exp(-(x-2*np.pi)**2/2/0.6**2)
				self.rpy[2] = np.arctan2(2*(1-np.cos(2*np.pi/10000*(i+1)))-2*(1-np.cos(x)),np.sin(2*2*np.pi/10000*(i+1))-np.sin(2*x))
				if key_press == 'escape':
					break
				time.sleep(0.005)

		if key_press == 'm':
			self.loc_mode = not self.loc_mode
			self.rpy[1] = 0.0
			if self.loc_mode:
				print("local mode")
			else:
				print("moving mode")
		
		if key_press == 'y':
			if self.auto_mode:
				self.auto_mode = False
				print("manual mode")
			else:
				self.auto_mode = True
				print("auto mode")

		if self.loc_mode:
			if key_press == 'up':
				self.rpy[1] += self.angle_step/2
				if self.rpy[1] > np.pi/6:
					self.rpy[1] = np.pi/6
				print("pitch angle", self.rpy[1])
			elif key_press == 'down':
				self.rpy[1] -= self.angle_step/2
				if self.rpy[1] < -np.pi/6:
					self.rpy[1] = -np.pi/6
				print("pitch angle", self.rpy[1])
		if key_press == 'escape':
			self.stop = 1
			self.start = False
			print("x=%s y=%s z=%s" % (self.pos[0], self.pos[1], self.pos[2]))



if __name__ == '__main__':

	remoter = KeyboardControl(mode=0)
	while remoter.stop == False:
		remoter.command.update()
		time.sleep(0.01)