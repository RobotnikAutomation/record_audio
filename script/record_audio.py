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

startRecord = False
FOLDER_NAME = ""

def callBackEndRecording(request): # if comething was publish in this service, stop recording
	response = TriggerResponse()
	response.success = True
	response.message = "Stop recording"
	global startRecord 
	startRecord = False
	rospy.loginfo(response.message)
	return response

def callBackStartRecording(request): # if comething was publish in this service, start recording
	response = TriggerResponse()
	response.success = True
	response.message = "Start recording"
	global startRecord 
	startRecord = True

	rospy.loginfo(response.message)

	return response

	
def record():
	# create folder where to put the audio file
	home = expanduser("~")
	pathFolder = home + "/outputMemory/" + FOLDER_NAME
	# create directory concerning the name of the user if does not exists
	try:
		os.stat(pathFolder)
	except:
		os.makedirs(pathFolder)
	# file name
	FORMAT = rospy.get_param('~format')
	date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
	path = pathFolder + "/outputAudio_" + date + "." + FORMAT

	# create listener
	MICROPHONE_RATE = rospy.get_param('~microphone_rate')
	inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, device=DEVICE)
	inp.setchannels(1)
	inp.setrate(MICROPHONE_RATE)
	inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
	inp.setperiodsize(1024)

	w = wave.open(path, 'w')
	w.setnchannels(1)
	w.setsampwidth(2)
	w.setframerate(88200)

	while startRecord == True:
		l, data = inp.read()
		a = numpy.fromstring(data, dtype='int16')
		w.writeframes(data)

if __name__ == '__main__':
	# init node
	rospy.init_node("record_audio")

	# get params name
	DEVICE = rospy.get_param('~device')
	start_record_srv = rospy.Service('~start', Trigger, callBackStartRecording)

	stop_record_srv = rospy.Service('~stop', Trigger, callBackEndRecording)


	# get the folder name
	FOLDER_NAME = rospy.get_param('~identity')


	# start node
	#rospy.spin()

	while not rospy.is_shutdown():

		if startRecord == True:
			record()


		rospy.sleep(1.0)



