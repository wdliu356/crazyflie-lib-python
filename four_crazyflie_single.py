
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
		self.force_stop = False
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
		self.rollreceived = 0
		self.pitchreceived = 0
		self.yawreceived = 0
		self.loc_mode = False
		self.torque_mode = False
		self.torque_shared = 0.0
		self.rolltorquereceived = 0.0
		self.frame_roll = 0.0
		self.yawfb = 0.0
		self.roll_v = 0.0
		self.pitch_v = 0.0
		self.yawfb_v = 0.0
		self.rollratefb_v = 0.0
		self.pitchratefb_v = 0.0
		self.yawratefb_v = 0.0

		# self.gain_name = ['vxKp', 'vxKi', 'vyKp', 'vyKi','vzKp','vzKi','zFactor',
		self.gain_name = [
					'roll_kp','roll_ki','roll_kd',
					'pitch_kp','pitch_ki','pitch_kd',
					'yaw_kp','yaw_ki','yaw_kd',
					'Ixx','Iyy','Izz','armLength',
					'roll_kp_f','roll_ki_f','roll_kd_f',
					'pitch_kp_f','pitch_ki_f','pitch_kd_f',
					'yaw_kp_f','yaw_ki_f','yaw_kd_f',
					'spring','Iyy_frame','Izz_frame']
		# self.gain_value = [3.5,0.001,3.5,0.001,10,20,0.6,
		# 			 0.5,-10.0,-225,0.6,10.0,300,
		# 			 0.04,0.1,2.0,0.0864,2.0011E-06,6.9007E-06,0.0001064235,0.041]
		# self.gain_value = [3.5,0.1,3.5,0.1,10,20,0.8,
		
		# for large quad
		self.gain_value = [
					 20.0,0.0,10.0,
					 20.0,0.0,10.0,
					 20.0,0.0,10.0,
					 6.3819E-05,9.8489e-05,0.00015404,0.055,
					 700.0,10.0,80.0,
					 500.0,10.0,80.0,
					 15.0,1.0, 5.0,
					 0.15,8.1398E-06,0.1065**2*0.0801*2+1.3349E-05+0.0002545*2
		]
		

		# # for small quad
		# self.gain_value = [
		# 			 4000.0,800.0,80.0,
		# 			 450.0,50.0,120.0,
		# 			 100.0, 5.0, 30.0,
		# 			 4.6929E-05,4.7145E-05,9.0062E-05,0.041,
		# 			 700.0,10.0,80.0,
		# 			 500.0,10.0,80.0,
		# 			 15.0,1.0, 5.0,
		# 			 0.6,1.0428E-06,1.2008E-05+3.5235E-06*2+2*0.07645**2*0.0098]
		
		### Tune pid on z direction
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
		# self._lg_estimate = LogConfig(name='stateEstimate', period_in_ms=10)
		# self._lg_thrust = LogConfig(name='thrust', period_in_ms=10)
		self._lg_stabilizer.add_variable('stabilizer.roll', 'float')
		self._lg_stabilizer.add_variable('stabilizer.pitch', 'float')
		self._lg_stabilizer.add_variable('stabilizer.yaw', 'float')
		# self._lg_estimate.add_variable('stabilizer.thrust', 'float')
		self._lg_stabilizer.add_variable('gyro.x', 'float')
		self._lg_stabilizer.add_variable('gyro.y', 'float')
		self._lg_stabilizer.add_variable('gyro.z', 'float')
		# self._lg_estimate.add_variable('stateEstimate.vx', 'float')
		# self._lg_estimate.add_variable('stateEstimate.vy', 'float')
		# self._lg_estimate.add_variable('stateEstimate.vz', 'float')
		# self._lg_estimate.add_variable('stabilizer.estimator','uint8_t')
		self._lg_state.add_variable('customizedCtl.rollTorque', 'float')
		self._lg_state.add_variable('customizedCtl.pitchTorque', 'float')
		self._lg_state.add_variable('customizedCtl.yawTorque', 'float')
		
		# self._lg_state.add_variable('customizedCtl.rollD', 'float')
		# self._lg_state.add_variable('customizedCtl.pitchD', 'float')
		# self._lg_state.add_variable('customizedCtl.yawD', 'float')

		self._lg_state.add_variable('customizedCtl.rollPID','float')
		self._lg_state.add_variable('customizedCtl.pitchPID','float')
		self._lg_state.add_variable('customizedCtl.yawPID','float')
		# self._lg_state.add_variable('customizedCtl.rollTorqueRecieved', 'float')
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

		# self._lg_thrust.add_variable('motor.m1', 'int32_t')
		# self._lg_thrust.add_variable('motor.m2', 'int32_t')
		# self._lg_thrust.add_variable('motor.m3', 'int32_t')
		# self._lg_thrust.add_variable('motor.m4', 'int32_t')

		# self._lg_supervisor = LogConfig(name='supervisor', period_in_ms=100)
		# self._lg_supervisor.add_variable('supervisor.info', 'uint16_t')

		
		


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

		# self._cf.log.add_config(self._lg_estimate)
		# self._lg_estimate.data_received_cb.add_callback(self._state_estimate_log_data)
		# self._lg_estimate.error_cb.add_callback(self._stab_log_error)
		# self._lg_estimate.start()

		# self._cf.log.add_config(self._lg_thrust)
		# self._lg_thrust.data_received_cb.add_callback(self._thrust_log_data)
		# self._lg_thrust.error_cb.add_callback(self._stab_log_error)
		# self._lg_thrust.start()

		# self._cf.log.add_config(self._lg_supervisor)
		# self._lg_supervisor.data_received_cb.add_callback(self._supervisor_log_data)
		# self._lg_supervisor.error_cb.add_callback(self._stab_log_error)
		# self._lg_supervisor.start()

		
			

		# except KeyError as e:
		# 	print('Could not start log configuration,'
		# 		  '{} not found in TOC'.format(str(e)))
		# except AttributeError:
		# 	print('Could not add Stabilizer log config, bad configuration.')


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


	def _thrust_log_data(self, timestamp, data, logconf):
		self.thrust1 = data["motor.m1"]
		self.thrust2 = data["motor.m2"]
		self.thrust3 = data["motor.m3"]
		self.thrust4 = data["motor.m4"]

	def _battery_log_data(self, timestamp, data, logconf):
		battery_data = round(data['pm.vbat'], 1)
		# if battery_data < 3.7:
		# 	print('Battery voltage of |CF %s| is: |3.1V(E)| --- %s V --- |4.2V(F)|' % (self.index, battery_data))
		print('Battery voltage of |CF %s| is:  %s V ' % (self.index, battery_data))
		self._lg_battery.data_received_cb.remove_callback(self._battery_log_data)


	def _stab_log_error(self, logconf, msg):
		print('Error when logging %s: %s' % (logconf.name, msg))

	def _state_log_data(self, timestamp, data, logconf):
		self.timest = timestamp
		# self.thrust = data["customizedCtl.thrust"]
		self.rolltorque = data["customizedCtl.rollTorque"]
		self.pitchtorque = data["customizedCtl.pitchTorque"]
		self.yawtorque = data["customizedCtl.yawTorque"]
		# self.rolltorquereceived = data["customizedCtl.rollD"]
		# self.pitchreceived = data["customizedCtl.pitchD"]
		# self.yawreceived = data["customizedCtl.yawD"]
		self.rollreceived = data["customizedCtl.rollPID"]
		self.pitchreceived = data["customizedCtl.pitchPID"]
		self.yawreceived = data["customizedCtl.yawPID"]
		# self.rolltorquereceived = data["customizedCtl.rollTorqueRecieved"]
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
		self.pitch = -data["stabilizer.pitch"]/180*np.pi
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
		# print("start",self.start)
		# if self.groundmode == True:
		# 	print("ground mode")
		# else:
		# 	print("air mode")
		# print("reset",self.reset)
		
		self._cf.commander.send_cus(self.rolld,self.pitchd,self.yawd,self.thrustd,self.start,self.groundmode,self.loc_mode,self.frame_roll,self.yawfb)


	def _torque_test(self):
		self._cf.commander.send_torque(self.torque_shared,self.start,self.force_stop,self.thrustd)
	
	def _direction_test(self):
		self._cf.commander.send_test(self.roll_v,self.pitch_v,self.yawfb_v,self.rollratefb_v,self.pitchratefb_v,self.yawratefb_v,self.start)


	def _stop_crazyflie(self):
		self.force_stop = True
		for i in range(10):
			self._cf.commander.send_cus(self.rolld,self.pitchd,self.yawd,self.thrustd,self.start,self.groundmode,self.loc_mode,self.frame_roll,self.yawfb)
			time.sleep(0.1)
			self._cf.commander.send_stop_setpoint()
			time.sleep(0.1)
		print("stop command sent")
		self._cf.close_link()

if __name__ == '__main__':
	uri = 'radio://0/80/2M'
	cf = SingleCF(uri, 1)
	# time.sleep(5)

