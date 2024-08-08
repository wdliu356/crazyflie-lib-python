
"""
Establish connection with single Crazyflie
"""
import logging
import time
import os
import sys
import csv
import numpy as np

from cflib.crazyflie.log import LogConfig
import cflib
from cflib.crazyflie import Crazyflie
from gimbal_platform_keyboard import KeyboardControl

logging.basicConfig(level=logging.ERROR)

class  SingleCF:
	def __init__(self, link_uri, index):

		self._cf = Crazyflie(rw_cache='./cache')
		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)
		self._cf.open_link(link_uri)
		self.index = index

		print('Connecting to %s' % link_uri)


	


	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)

		self._cf.param.add_update_callback(group='motorPowerSet', name=None,cb=self._param_callback)
		#read battery voltage
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat' ,'float')
		self._cf.log.add_config(self._lg_battery)
		self._lg_battery.data_received_cb.add_callback(self._battery_log_data)
		self._lg_battery.start()
		
	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
		# self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)


	def _param_callback(self, name, value):
		# print('Readback: {0}={1}'.format(name, value))
		self._cf.param.remove_update_callback(self._param_callback)

		

	def _connection_failed(self, link_uri, msg):
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		print('Disconnected from %s' % link_uri)

	def _update_motors(self):
		self._cf.commander.send_cus(self.vx,self.vy,self.vz,self.yawrate,self.yaw,self.groundmode,self.reset)

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
		# time.sleep(0.1)
		self._cf.close_link()


if __name__ == '__main__':
	cflib.crtp.init_drivers(enable_debug_driver=False)
	# Scan for Crazyflies and use the first one found
	print('Scanning interfaces for Crazyflies...')
	available = cflib.crtp.scan_interfaces()
	print('Crazyflies found:')
	for i in available:
		print(i[0])
	time.sleep(0.25)
	keyboard = KeyboardControl(control_mode='attitude')
	qc = SingleCF('radio://0/80/2M/E7E7E7E7E7', 0)
	time.sleep(0.25)
	qc._cf.param.set_value('motorPowerSet.enable', '1')
	for i in range(50):
		# print(i)
		keyboard.command.update()
		if keyboard.stop == True:
			break
		pwm = int(65535/50*(i+1))
		print(pwm)
		# qc._cf.param.set_value('motorPowerSet.m1', str(pwm))
		# qc._cf.param.set_value('motorPowerSet.m2', str(pwm))
		# qc._cf.param.set_value('motorPowerSet.m3', str(pwm))
		qc._cf.param.set_value('motorPowerSet.m4', str(pwm))
		time.sleep(1)
	# qc._cf.param.set_value('motorPowerSet.m1', '0')
	# qc._cf.param.set_value('motorPowerSet.m2', '0')
	# qc._cf.param.set_value('motorPowerSet.m3', '0')
	qc._cf.param.set_value('motorPowerSet.m4', '0')