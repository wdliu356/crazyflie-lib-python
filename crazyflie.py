
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

logging.basicConfig(level=logging.ERROR)

class  SingleCF:
	def __init__(self, link_uri, index):

		self._cf = Crazyflie(rw_cache='./cache')
		self._cf.connected.add_callback(self._connected)
		self._cf.disconnected.add_callback(self._disconnected)
		self._cf.connection_failed.add_callback(self._connection_failed)
		self._cf.connection_lost.add_callback(self._connection_lost)
		self._cf.open_link(link_uri)
		print('Connecting to %s' % link_uri)
		self.index = index
		self.quat = np.array([1 ,0 ,0 ,0])
		self.thrustd = 0.0
		self.torque = np.array([0.0,0.0,0.0])
		self.start = False
		self.m1 = 0
		self.m2 = 0
		self.m3 = 0
		self.m4 = 0
		self.u1 = 0
		self.u2 = 0
		self.u3 = 0
		self.u4 = 0


	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)


		# modify PID gains parameters
		time.sleep(0.25)
		self._cf.param.add_update_callback(group='customizedPid', name=None,
										   cb=self._param_callback)
		
		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat' ,'float')
		self._lg_motor = LogConfig(name='motor', period_in_ms=10)
		
		self._lg_motor.add_variable('motor.m1pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m2pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m3pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m4pwm', 'uint16_t')
		self._lg_motor.add_variable('powerDist.m1force', 'float')
		self._lg_motor.add_variable('powerDist.m2force', 'float')
		self._lg_motor.add_variable('powerDist.m3force', 'float')
		self._lg_motor.add_variable('powerDist.m4force', 'float')
		# try:
		self._cf.log.add_config(self._lg_battery)
		self._lg_battery.data_received_cb.add_callback(self._battery_log_data)
		self._lg_battery.start()

		self._cf.log.add_config(self._lg_motor)
		self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
		self._lg_motor.start()


	def _param_callback(self, name, value):
		# print('Readback: {0}={1}'.format(name, value))
		self._cf.param.remove_update_callback(self._param_callback)


	def _supervisor_log_data(self, timestamp, data, logconf):
		# print('The supervisor info is %s' % data['supervisor.info'])
		if data['supervisor.info'] == 1:
			print('The system can be armed and will accept an arming command')
		elif data['supervisor.info'] == 2:
			print('The system is currently armed')
		elif data['supervisor.info'] == 2+4:
			print('The system is configured to automatically arm')
		elif data['supervisor.info'] == 2+4+8:
			print('The Crazyflie is ready to fly')
		elif data['supervisor.info'] == 2+4+8+16:
			print('The Crazyflie is currently flying')
		elif data['supervisor.info'] == 2+4+8+16+32:
			print('The crazyflie is up side down/tumbling')
		elif data['supervisor.info'] == 2+4+8+16+32+64:
			print('The Crazyflie is locked and must be restarted')


	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		# if battery_data < 3.7:
		# 	print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
		print('Battery voltage of |CF %s| is:  %s V ' % (self.index, battery_data))
		self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)


	def _stab_log_error(self, logconf, msg):
		print('Error when logging %s: %s' % (logconf.name, msg))

		
	def _motor_log_data(self, timestamp, data, logconf):
		self.m1 = data["motor.m1pwm"]
		self.m2 = data["motor.m2pwm"]
		self.m3 = data["motor.m3pwm"]
		self.m4 = data["motor.m4pwm"]
		self.u1 = data["powerDist.m1force"]
		self.u2 = data["powerDist.m2force"]
		self.u3 = data["powerDist.m3force"]
		self.u4 = data["powerDist.m4force"]

	
	def _connection_failed(self, link_uri, msg):
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		print('Disconnected from %s' % link_uri)

	def _update_motors(self):		
		self._cf.commander.send_torque_control(self.torque[0],self.torque[1],self.torque[2],self.thrustd,self.start)


	def _stop_crazyflie(self):
		self.force_stop = True
		for i in range(10):
			self._cf.commander.send_torque_control(self.torque[0],self.torque[1],self.torque[2],self.thrustd,self.start)
			time.sleep(0.1)
			self._cf.commander.send_stop_setpoint()
			time.sleep(0.1)
		print("stop command sent")
		self._cf.close_link()

if __name__ == '__main__':
	uri = 'radio://0/80/2M'
	cf = SingleCF(uri, 1)
	# time.sleep(5)

