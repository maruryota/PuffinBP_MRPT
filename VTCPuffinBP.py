#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file VTCPuffinBP.py
 @brief ModuleDescription
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist
import zmq
import math
import json
import time


# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
vtcpuffinbp_spec = ["implementation_id", "VTCPuffinBP",
		 "type_name",         "VTCPuffinBP",
		 "description",       "ModuleDescription",
		 "version",           "1.0.0",
		 "vendor",            "maruryota",
		 "category",          "Category",
		 "activity_type",     "STATIC",
		 "max_instance",      "1",
		 "language",          "Python",
		 "lang_type",         "SCRIPT",
		 "conf.default.max_acc", "1",
		 "conf.default.max_rot_acc", "1",
		 "conf.default.max_vel", "1",
		 "conf.default.max_rot_vel", "1",
		 "conf.default.host", "localhost",
		 "conf.default.port", "54323",

		 "conf.__widget__.max_acc", "text",
		 "conf.__widget__.max_rot_acc", "text",
		 "conf.__widget__.max_vel", "text",
		 "conf.__widget__.max_rot_vel", "text",
		 "conf.__widget__.host", "text",
		 "conf.__widget__.port", "text",

         "conf.__type__.max_acc", "double",
         "conf.__type__.max_rot_acc", "double",
         "conf.__type__.max_vel", "double",
         "conf.__type__.max_rot_vel", "double",
         "conf.__type__.host", "string",
         "conf.__type__.port", "string",

		 ""]
# </rtc-template>

##
# @class VTCPuffinBP
# @brief ModuleDescription
#
#
class VTCPuffinBP(OpenRTM_aist.DataFlowComponentBase):

	##
	# @brief constructor
	# @param manager Maneger Object
	#
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_targetVelocity = OpenRTM_aist.instantiateDataType(RTC.TimedVelocity2D)
		"""
		"""
		self._targetVelocityIn = OpenRTM_aist.InPort("targetVelocity", self._d_targetVelocity)
		self._d_odometry = OpenRTM_aist.instantiateDataType(RTC.TimedVelocity2D)
		"""
		"""
		self._odometryOut = OpenRTM_aist.OutPort("odometry", self._d_odometry)
		self._d_currentPose = OpenRTM_aist.instantiateDataType(RTC.TimedPose2D)
		"""
		"""
		self._currentPoseOut = OpenRTM_aist.OutPort("currentPose", self._d_currentPose)





		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		"""
		
		 - Name:  max_acc
		 - DefaultValue: 1
		"""
		self._max_acc = [1]
		"""
		
		 - Name:  max_rot_acc
		 - DefaultValue: 1
		"""
		self._max_rot_acc = [1]
		"""
		
		 - Name:  max_vel
		 - DefaultValue: 1
		"""
		self._max_vel = [1]
		"""
		
		 - Name:  max_rot_vel
		 - DefaultValue: 1
		"""
		self._max_rot_vel = [1]
		"""
		
		 - Name:  host
		 - DefaultValue: localhost
		"""
		self._host = ['localhost']
		"""
		
		 - Name:  port
		 - DefaultValue: 54323
		"""
		self._port = ['54323']

		# </rtc-template>



	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry()
	#
	# @return RTC::ReturnCode_t
	#
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		self.bindParameter("max_acc", self._max_acc, "1")
		self.bindParameter("max_rot_acc", self._max_rot_acc, "1")
		self.bindParameter("max_vel", self._max_vel, "1")
		self.bindParameter("max_rot_vel", self._max_rot_vel, "1")
		self.bindParameter("host", self._host, "localhost")
		self.bindParameter("port", self._port, "54323")

		# Set InPort buffers
		self.addInPort("targetVelocity",self._targetVelocityIn)

		# Set OutPort buffers
		self.addOutPort("odometry",self._odometryOut)
		self.addOutPort("currentPose",self._currentPoseOut)

		# Set service provider to Ports

		# Set service consumers to Ports

		# Set CORBA Service Ports
		print("finish initialization")

		return RTC.RTC_OK

	##
	#
	# The finalize action (on ALIVE->END transition)
	# formaer rtc_exiting_entry()
	#
	# @return RTC::ReturnCode_t
	
	#
	def onFinalize(self):
	
		return RTC.RTC_OK

	###
	##
	## The startup action when ExecutionContext startup
	## former rtc_starting_entry()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK

	###
	##
	## The shutdown action when ExecutionContext stop
	## former rtc_stopping_entry()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK

	##
	#
	# The activated action (Active state entry action)
	# former rtc_active_entry()
	#
	# @param ec_id target ExecutionContext Id
	#
	# @return RTC::ReturnCode_t
	#
	#
	def onActivated(self, ec_id):
		
		cxt=zmq.Context()
		sock=cxt.socket(zmq.REQ)
		sock.setsockopt(zmq.REQ_CORRELATE, True);
		sock.setsockopt(zmq.REQ_RELAXED, True);
		sock.setsockopt(zmq.RCVTIMEO, 1000);
		sock.setsockopt(zmq.SNDTIMEO, 1000);
		self.endpoint ="PuffinBP_2"
		address = f"tcp://{self._host[0]}:{self._port[0]}"
		print(f"start connecting to {address}")
		sock.connect(address)
		self.sock = sock
		print("finish activation")
		self.odometry = {
			"x": 0,
			"y": 0,
			"heading": 0
		}
		self.avg_exec_sec = 0.006  # initial average execution second, delta time
		self.run_times = []
		self.start_time = 0
		self.end_time = 0
		return RTC.RTC_OK

	###
	##
	## The deactivated action (Active state exit action)
	## former rtc_active_exit()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onDeactivated(self, ec_id):
	#
	#	return RTC.RTC_OK

	##
	#
	# The execution action that is invoked periodically
	# former rtc_active_do()
	#
	# @param ec_id target ExecutionContext Id
	#
	# @return RTC::ReturnCode_t
	#
	#
	def onExecute(self, ec_id):
		self.start_time = time.time()
		if self._targetVelocityIn.isNew():
			timed_velocity2d_in = self._targetVelocityIn.read()
			input_velocity2d = timed_velocity2d_in.data
			os = {
			"Type": "ActorMsg",
			"Endpoint": self.endpoint
			}
		
			command = {
				'CmdType':'VW',
				'V': float(input_velocity2d .vx)*100,
				'W': round(float(input_velocity2d .va)*180 / math.pi, 2)
			}
			print(command)
			list_req = [
				json.dumps(os, indent=2).encode("utf-8"), 
				json.dumps(command, indent=2).encode("utf-8")
			]

			self.sock.send_multipart(list_req)
			res = self.sock.recv()
			dict_res = json.loads(res)
			assert dict_res["Result"] == "OK"
			print(dict_res)

			self._d_currentPose.data.heading += input_velocity2d.va * self.avg_exec_sec
			self._d_currentPose.data.position.x += input_velocity2d.vx * math.cos(input_velocity2d.va) * self.avg_exec_sec
			self._d_currentPose.data.position.y += input_velocity2d.vx * math.sin(input_velocity2d.va) * self.avg_exec_sec
			self._currentPoseOut.write()

		self.end_time = time.time() - self.start_time
		self.run_times.append(self.end_time)
		self.avg_exec_sec = sum(self.run_times[-10:]) / len(self.run_times[-10:])
		print(f"average execution time: {self.avg_exec_sec}")
		return RTC.RTC_OK

	###
	##
	## The aborting action when main logic error occurred.
	## former rtc_aborting_entry()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK

	###
	##
	## The error action in ERROR state
	## former rtc_error_do()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK

	###
	##
	## The reset action that is invoked resetting
	## This is same but different the former rtc_init_entry()
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK

	###
	##
	## The state update action that is invoked after onExecute() action
	## no corresponding operation exists in OpenRTm-aist-0.2.0
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##

	##
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK

	###
	##
	## The action that is invoked when execution context's rate is changed
	## no corresponding operation exists in OpenRTm-aist-0.2.0
	##
	## @param ec_id target ExecutionContext Id
	##
	## @return RTC::ReturnCode_t
	##
	##
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK




def VTCPuffinBPInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=vtcpuffinbp_spec)
    manager.registerFactory(profile,
                            VTCPuffinBP,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    VTCPuffinBPInit(manager)

    # Create a component
    comp = manager.createComponent("VTCPuffinBP")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

