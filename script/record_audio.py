#!/usr/bin/env python3
#coding: utf-8

'''
author: Andres Font Guzman
date: 14.12.2022
description: Node used to record sound from a device using a rospy node. Work with alsaaudio lib.

'''

import os, rospy, datetime, alsaaudio, wave, numpy
from os.path import expanduser
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from robotnik_msgs.msg import RecordStatus
from robotnik_msgs.srv import Record, RecordResponse


def callBackRecordService(request):
	global action
	global max_time_record
	global time_start_record
	global file_name
	global inp
	global wave_interface
	response = RecordResponse()
	if (request.action.upper() == request.ACTION_RECORD):
		if(action == "Idle"):
			date = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
			
			if request.max_time<=0:
				max_time_record = 0
			else:
				max_time_record=request.max_time

			if request.file_name=="":
				file_name=date+"."+file_format
			else:
				file_name=request.file_name+"."+file_format
			try:
				os.stat(folder_path)
			except:
				os.makedirs(folder_path)
			
			try:
				
				inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, device=device)
				inp.setchannels(1)
				inp.setrate(microphone_rate)
				inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
				inp.setperiodsize(1024)

				wave_interface = wave.open(folder_path+"/"+file_name, 'w')
				wave_interface.setnchannels(1)
				wave_interface.setsampwidth(2)
				wave_interface.setframerate(88200)
			except Exception as e:
				response.success = False
				response.message = "Exception to start recording. Ex: " + str(e)
				return response
			time_start_record = rospy.Time.now()
			rospy.loginfo("Starting recording")
			action="Recording"
		else:
			response.success = False
			response.message = "The node is not in Idle state"
	elif (request.action.upper() == request.ACTION_SAVE):
		if(action == "Recording"):
			action="Idle"
		else:
			response.success = False
			response.message = "The node is not in Recording state"
	elif (request.action.upper() == request.ACTION_STOP):
		if(action == "Recording"):	
			action="Idle"
		else:
			response.success = False
			response.message = "The node is not in Recording state"
	else:
		response.success = False
		response.message = "Action " + request.action + " is not defined."
	return response


def record():
	global inp
	global wave_interface
	l, data = inp.read()
	a = numpy.fromstring(data, dtype='int16')
	wave_interface.writeframes(data)

if __name__ == '__main__':
	# init node
	rospy.init_node("record_audio")
	global device
	global max_time_record
	global folder_path
	global file_name
	global record_state
	global action
	global file_format
	global microphone_rate
	global inp
	global wave_interface
	global time_start_record

	max_time_record = 0
	folder_path = ""
	file_name = ""
	record_state = RecordStatus()
	action = "Idle"
	file_format = "mp3"
	microphone_rate = "96000"
	device = "default"
	inp = None
	wave_interface = None
	time_start_record = None

	# get params name
	device = rospy.get_param('~device', device)
	folder_path = expanduser("~") + "/record"
	folder_path = rospy.get_param('~folder_path', folder_path)
	# file name
	file_format = rospy.get_param('~format', file_format)
	if(file_format!="mp3" or file_format!="wav"):
		file_format = "mp3"
	microphone_rate = rospy.get_param('~microphone_rate', microphone_rate)
	# Services
	record_service = rospy.Service('~record', Record, callBackRecordService)
	# Topic
	record_state_pub = rospy.Publisher('~status', RecordStatus, queue_size=1)
	action = "Idle"
	record_state.state_description = action
	record_state.recording = False
	record_state.recording_time = 0

	while not rospy.is_shutdown():
		record_state.state_description = action
		if action == "Recording":
			current_time = time_end = rospy.Time.now()
			seconds_diff = current_time - time_start_record
			if(max_time_record <= 0 or int(seconds_diff.to_sec())<=max_time_record):
				record()
			else:
				action = "Idle"
			record_state.recording = True
			record_state.state_description += " " + str(int(seconds_diff.to_sec())) + " seg."
			record_state.recording_time = int(seconds_diff.to_sec())
			rospy.loginfo(record_state.state_description)
		else:
			record_state.state_description = action
			record_state.recording = False
			record_state.recording_time = 0
		record_state_pub.publish(record_state)
		rospy.sleep(1/int(microphone_rate))



