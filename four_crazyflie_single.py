
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
		self.rolld = 0.0
		self.pitchd = 0.0
		self.yawd = 0.0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0.0
		self.groundmode = True
		self.start = False
		self.reset = False
		self.timest = 0
		self.vx = 0.0
		self.vy = 0.0
		self.vz = 0.0
		self.yawrate = 0.0
		self.vxfb = 0
		self.vyfb = 0
		self.vzfb = 0
		self.rollratefb = 0
		self.pitchratefb = 0
		self.yawratefb = 0
		self.thrust = 0
		self.yawfb = 0
		self.rolltorque = 0
		self.pitchtorque = 0
		self.yawtorque = 0
		self.rollref = 0
		self.pitchref = 0
		self.yawref = 0
		self.m1 = 0
		self.m2 = 0
		self.m3 = 0
		self.m4 = 0
		self.u1 = 0
		self.u2 = 0
		self.u3 = 0
		self.u4 = 0
		self.m1req = 0
		self.m2req = 0
		self.m3req = 0
		self.m4req = 0
		self.thrust1 = 0
		self.thrust2 = 0
		self.thrust3 = 0
		self.thrust4 = 0

		# self.gain_name = ['vxKp', 'vxKi', 'vyKp', 'vyKi','vzKp','vzKi','zFactor',
		self.gain_name = [
					'roll_kp','roll_ki','roll_kd',
					'pitch_kp','pitch_ki','pitch_kd',
					'yaw_kp','yaw_ki','yaw_kd',
					'mass','Ixx','Iyy','Izz','armLength']
		# self.gain_value = [3.5,0.001,3.5,0.001,10,20,0.6,
		# 			 0.5,-10.0,-225,0.6,10.0,300,
		# 			 0.04,0.1,2.0,0.0864,2.0011E-06,6.9007E-06,0.0001064235,0.041]
		# self.gain_value = [3.5,0.1,3.5,0.1,10,20,0.8,
		
		# self.gain_value = [
		# 			 0.8,0.3,-900.0,0.8,1.0,-850.0,0.03,0.02,-10.0,
		# 			 0.0864,2.0011E-06,6.9007E-06,0.0001064235,0.041] # tune here
		self.gain_value = [
					 0.8, 0.3, -1200.0,
					 0.8, 1.0, -850.0,
					 0.03, 0.02, -15.0,
					 0.0864,2.0011E-06,6.9007E-06,0.0001064235,0.041] # tune here
		print("start setting param")
		for n in self.gain_name:
			print(n)
			ind = self.gain_name.index(n)
			self._cf.param.set_value('customizedPid.{}'.format(n), '{}'.format(self.gain_value[ind]))
		self._cf.param.set_value('stabilizer.controller', '6')
		print("finished setting param")
		
		
		


	def _connected(self, link_uri):

		print('Connected to %s' % link_uri)


		# modify PID gains parameters
		time.sleep(0.25)
		self._cf.param.add_update_callback(group='customizedPid', name=None,
										   cb=self._param_callback)
		
		# self._cf.param.remove_update_callback(group='controller_tune', name=n,
		# 									cb=self._param_callback)


		self._lg_battery = LogConfig(name='pm', period_in_ms=1000)
		self._lg_battery.add_variable('pm.vbat' ,'float')
		self._lg_state = LogConfig(name='state', period_in_ms=10)
		self._lg_motor = LogConfig(name='motor', period_in_ms=10)
		self._lg_stabilizer = LogConfig(name='stabilizer', period_in_ms=10)
		self._lg_estimate = LogConfig(name='stateEstimate', period_in_ms=10)
		self._lg_thrust = LogConfig(name='thrust', period_in_ms=10)
		self._lg_stabilizer.add_variable('stabilizer.roll', 'float')
		self._lg_stabilizer.add_variable('stabilizer.pitch', 'float')
		self._lg_stabilizer.add_variable('stabilizer.yaw', 'float')
		self._lg_estimate.add_variable('stabilizer.thrust', 'float')
		self._lg_stabilizer.add_variable('gyro.x', 'float')
		self._lg_stabilizer.add_variable('gyro.y', 'float')
		self._lg_stabilizer.add_variable('gyro.z', 'float')
		self._lg_estimate.add_variable('stateEstimate.vx', 'float')
		self._lg_estimate.add_variable('stateEstimate.vy', 'float')
		self._lg_estimate.add_variable('stateEstimate.vz', 'float')
		self._lg_estimate.add_variable('stabilizer.estimator','uint8_t')
		self._lg_state.add_variable('customizedCtl.rollTorque', 'float')
		self._lg_state.add_variable('customizedCtl.pitchTorque', 'float')
		self._lg_state.add_variable('customizedCtl.yawTorque', 'float')
		
		self._lg_state.add_variable('customizedCtl.rollD', 'float')
		self._lg_state.add_variable('customizedCtl.pitchD', 'float')
		self._lg_state.add_variable('customizedCtl.yawD', 'float')
		# self._lg_state.add_variable('customizedCtl.mass', 'float')
		# self._lg_state.add_variable('customizedCtl.g', 'float')
		# self._lg_state.add_variable('customizedCtl.zFactor', 'float')
		# self._lg_state.add_variable('ctrltarget.zMode', 'int16_t')
		self._lg_motor.add_variable('motor.m1pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m2pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m3pwm', 'uint16_t')
		self._lg_motor.add_variable('motor.m4pwm', 'uint16_t')
		self._lg_motor.add_variable('powerDist.m1force', 'float')
		self._lg_motor.add_variable('powerDist.m2force', 'float')
		self._lg_motor.add_variable('powerDist.m3force', 'float')
		self._lg_motor.add_variable('powerDist.m4force', 'float')

		# self._lg_estimate.add_variable('motor.m1req', 'int32_t')
		# self._lg_estimate.add_variable('motor.m2req', 'int32_t')
		# self._lg_estimate.add_variable('motor.m3req', 'int32_t')
		# self._lg_estimate.add_variable('motor.m4req', 'int32_t')

		self._lg_thrust.add_variable('motor.m1', 'int32_t')
		self._lg_thrust.add_variable('motor.m2', 'int32_t')
		self._lg_thrust.add_variable('motor.m3', 'int32_t')
		self._lg_thrust.add_variable('motor.m4', 'int32_t')
		
		


		# try:
		self._cf.log.add_config(self._lg_battery)
		self._lg_battery.data_received_cb.add_callback(self._battery_log_data)
		self._lg_battery.start()

		self._cf.log.add_config(self._lg_state)
		self._lg_state.data_received_cb.add_callback(self._state_log_data)
		self._lg_state.error_cb.add_callback(self._stab_log_error)
		self._lg_state.start()

		self._cf.log.add_config(self._lg_motor)
		self._lg_motor.data_received_cb.add_callback(self._motor_log_data)
		self._lg_motor.start()

		self._cf.log.add_config(self._lg_stabilizer)
		self._lg_stabilizer.data_received_cb.add_callback(self._stabilizer_log_data)
		self._lg_stabilizer.error_cb.add_callback(self._stab_log_error)
		self._lg_stabilizer.start()

		self._cf.log.add_config(self._lg_estimate)
		self._lg_estimate.data_received_cb.add_callback(self._state_estimate_log_data)
		self._lg_estimate.error_cb.add_callback(self._stab_log_error)
		self._lg_estimate.start()

		self._cf.log.add_config(self._lg_thrust)
		self._lg_thrust.data_received_cb.add_callback(self._thrust_log_data)
		self._lg_thrust.error_cb.add_callback(self._stab_log_error)
		self._lg_thrust.start()
			

		# except KeyError as e:
		# 	print('Could not start log configuration,'
		# 		  '{} not found in TOC'.format(str(e)))
		# except AttributeError:
		# 	print('Could not add Stabilizer log config, bad configuration.')


	def _param_callback(self, name, value):
		# print('Readback: {0}={1}'.format(name, value))
		self._cf.param.remove_update_callback(self._param_callback)


	def _thrust_log_data(self, timestamp, data, logconf):
		self.thrust1 = data["motor.m1"]
		self.thrust2 = data["motor.m2"]
		self.thrust3 = data["motor.m3"]
		self.thrust4 = data["motor.m4"]

	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		if battery_data < 3.7:
			print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
		# print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
		# self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)


	def _stab_log_error(self, logconf, msg):
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _state_log_data(self, timestamp, data, logconf):
		self.timest = timestamp
		# self.thrust = data["customizedCtl.thrust"]
		self.rolltorque = data["customizedCtl.rollTorque"]
		self.pitchtorque = data["customizedCtl.pitchTorque"]
		self.yawtorque = data["customizedCtl.yawTorque"]
		self.rollref = data["customizedCtl.rollD"]
		self.pitchref = data["customizedCtl.pitchD"]
		self.yawref = data["customizedCtl.yawD"]
		# print('Thrust of |CF %s| is: %s' % (self.index, self.thrust))
		# print("mass is %s" % data["customizedCtl.mass"])
		# print("gravity is %s" % data["customizedCtl.g"])
		# print("zFactor is %s" % data["customizedCtl.zFactor"])
		# print("zMode is %s" % data["ctrltarget.zMode"])
		
	def _motor_log_data(self, timestamp, data, logconf):
		self.m1 = data["motor.m1pwm"]
		self.m2 = data["motor.m2pwm"]
		self.m3 = data["motor.m3pwm"]
		self.m4 = data["motor.m4pwm"]
		self.u1 = data["powerDist.m1force"]
		self.u2 = data["powerDist.m2force"]
		self.u3 = data["powerDist.m3force"]
		self.u4 = data["powerDist.m4force"]

	def _stabilizer_log_data(self, timestamp, data, logconf):
		self.roll = data["stabilizer.roll"]/180*np.pi
		self.pitch = data["stabilizer.pitch"]/180*np.pi
		self.yawfb = data["stabilizer.yaw"]/180*np.pi
		# print('the feedback yaw angle is %s' % self.yawfb)
		self.rollratefb = data["gyro.x"]/180*np.pi
		self.pitchratefb = data["gyro.y"]/180*np.pi
		self.yawratefb = data["gyro.z"]/180*np.pi

	def _state_estimate_log_data(self, timestamp, data, logconf):
		# pass
		self.vxfb = data["stateEstimate.vx"]
		self.vyfb = data["stateEstimate.vy"]
		self.vzfb = data["stateEstimate.vz"]
		self.thrust = data["stabilizer.thrust"]
		# print('the estimator type is %s' % data["stabilizer.estimator"])
		# self.m1req = data["motor.m1req"]
		# self.m2req = data["motor.m2req"]
		# self.m3req = data["motor.m3req"]
		# self.m4req = data["motor.m4req"]

		

	def _connection_failed(self, link_uri, msg):
		print('Connection to %s failed: %s' % (link_uri, msg))

	def _connection_lost(self, link_uri, msg):
		print('Connection to %s lost: %s' % (link_uri, msg))

	def _disconnected(self, link_uri):
		print('Disconnected from %s' % link_uri)

	def _update_motors(self):
		# self._cf.commander.send_cus(self.vx,self.vy,self.vz,self.yawrate,self.yaw,self.groundmode,self.reset)
		self._cf.commander.send_cus(self.rolld,self.pitchd,self.yawd,self.yawrate,self.thrustd,self.start,self.reset,self.groundmode)

	def _stop_crazyflie(self):
		self._cf.commander.send_stop_setpoint()
		# time.sleep(0.1)
		self._cf.close_link()

