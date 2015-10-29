#!/usr/bin/env python

"""DOCSTRING"""
import rospy
from geometry_msgs.msg import Twist, Vector3
import cv2
import followDot
# import ridgeRegression
import numpy as np


# def aveList(listInput):
# 	""" finds average of a list
# 			inputs: listInput - list of integers or floats
# 			outputs: average of all items in list"""

# 	return sum(listInput)/len(listInput)

def sendToNeato(pub, twist, currentYaw, currentPitch):
	""" sends yaw and pitch to Neato 
			inputs: pub - publisher to Neato 
							twist - Twist object for Neato
							currentYaw - average of previous 12 yaw readings
							currentPitch - average of previous 12 pitch readings"""

	twist.angular.z = (currentYaw-400) * .001
	twist.linear.x = (currentPitch-400) * .001
	pub.publish(self.twist)

def manipulateImage(im):
	"""compresses, gray-scales and reshapes an image to first a 24x24 image, then a 576x1 asarray
			inputs: im - image to be manipulated
			outputs: reshaped - 576-item long arry which represents the grayscale values of the photo"""

	compressed = followDot.compressImage(im)
	compressed = np.asarray(compressed, dtype=np.uint8)
	gray = cv2.cvtColor(compressed, cv2.COLOR_BGR2GRAY )
	reshaped = np.reshape(gray, 576)
	return reshaped

def updateAverage(previous12values, currentYaw, currentPitch):
	""" updates the calculation of the moving average of Yaw and pitch
			inputs - previous12values list of two lists, first of which is the most recent 12 yaw readings, not including the one just taken,
																second is the same list, but of pitch readings
							 currentYaw - most recent yaw reading
							 currentPitch - most recent pitch reading"""

	del previous12values[0][0]
	del previous12values[1][0]  
	previous12values[0].append(yaw)
	previous12values[1].append(pitch)
	currentYaw = aveList(previous12values[0])
	currentPitch = aveList(previous12values[1])
	print 'yaw', currentYaw, 'pitch', currentPitch
	return newYaw, newPitch, previous12valeus


# if __name__ == "__main__":
#calculate ridge model
rospy.init_node('CV')
ridge = ridgeRegression.get_ridge_model(.1)

#initialize camera
camera = followDot.initializeCamera()

#initialize face classifier
face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

#initialize lists to hold previous yaw and pitch values
previous12values =  ([0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0])

#initialize publisher and twist for neato
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
twist = Twist()

#enter run loop
running = True
while running == True:
	#reads camera
	ret, frame = camera.read()
	
	# detects faces
	faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
	
	#draws rectangle on face
	for (x,y,w,h) in faces:
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))

	# see if there is a face in the image, enter try if there is
	try:
		cropped = followDot.cropImage(frame, faces[0])
		reshaped = manipulateImage(cropped)    
		yaw, pitch =  ridgeRegression.ridge_predict([reshaped],ridge)
		previous12values, newPitch, newYaw = updateAverage(previous12values, yaw, pitch)
		sentToNeato(pub, twist, newYaw, newPitch)

	except IndexError: #When no face is detected, stop neato
		twist.angular.z = 0
		twist.linear.x = 0
		pub.publish(self.twist)

	#show the camera with face detection square 
	cv2.imshow('frame',frame)
	#waits until user presses q to start
	if cv2.waitKey(1) & 0xFF == ord('q'):
		ready=True 
		cv2.waitKey(1)
		running = False
