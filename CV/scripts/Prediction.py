#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
import cv2
from ImageManipulation import ImageManipulation

from RidgeRegression import RidgeModel

import numpy as np


class Predictor(object):

  def __init__(self):
    rospy.init_node('CV')
    self.ImManipulator = ImageManipulation()
    ridge = RidgeModel()
    self.ridge = ridge.ridge
    # faces = ()
    # while faces == ():
    #   faces, frame = self.ImManipulator.detectFaces()
    # self.name = self.predictFace(faces, self.ridge)
    self.previous12values =  ([0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0])

    #initialize publisher and twist for neato
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # self.pubYaw = rospy.Publisher('/yaw', yaw, queue_size=10)
    # self.pubPitch = rospy.Publisher('/pitch', pitch, queue_size=10)
    self.twist = Twist()
    print "initialized"

  def avgList(self, listInput):
    """ finds average of a list
        inputs: listInput - list of integers or floats
        outputs: average of all items in list"""

    return sum(listInput)/len(listInput)

  def sendToNeato(self, currentYaw, currentPitch):
    """ sends yaw and pitch to Neato 
        inputs: pub - publisher to Neato 
                twist - Twist object for Neato
                currentYaw - average of previous 12 yaw readings
                currentPitch - average of previous 12 pitch readings"""

    self.twist.angular.z = (currentYaw-400) * -0.005
    # Multiply by -1 to invert direction. Multiply by .001 to make much smaller. Add .8 to adjust range from -.8 -> 0  to 0->.8
    self.twist.linear.x = ((currentPitch) * -.001) + .8
    # print self.twist.linear.x
    # print ('pitch', currentPitch)
    self.pub.publish(self.twist)
    # self.pubYaw.publish(currentYaw)
    # self.pubPitch.publish(currentPitch)

  def updateAverage(self, currentYaw, currentPitch):
    """ updates the calculation of the moving average of Yaw and pitch
        inputs - previous12values list of two lists, first of which is the most recent 12 yaw readings, not including the one just taken,
                                  second is the same list, but of pitch readings
                 currentYaw - most recent yaw reading
                 currentPitch - most recent pitch reading"""

    del self.previous12values[0][0]
    del self.previous12values[1][0]  
    self.previous12values[0].append(currentYaw)
    self.previous12values[1].append(currentPitch)
    newYaw = self.avgList(self.previous12values[0])
    newPitch = self.avgList(self.previous12values[1])
    return newYaw, newPitch

  def predictFace(faces, clf, pca):
    """uses facial recognition model to predict the user's name"
    inputs: faces - list of detected faces
            clf - facial recognition classifier
            pca - Principle component analysis used for classifying faces"""
    cropped = followDot.cropImage(frame, faces)
    reshaped = manipulateImage(cropped)
    X_test_pca = pca.transform([reshaped])
    number = clf.predict(X_test_pca)
    with open('nameToNumber.txt','r') as inf:
      numberToName = eval(inf.read())
    name = numberToName[number[0]]
    print "Hello, " + name 
    return name

  def run(self):
    im = self.ImManipulator.detectFaces()
    if self.ImManipulator.faces != ():
      im = self.ImManipulator.cropImage(im)
      im = self.ImManipulator.reshapeImage(im)
      yaw, pitch = self.ridge.predict(im)
      newYaw, newPitch = self.updateAverage(yaw, pitch)
      self.sendToNeato(newYaw, newPitch)
    else:
      # if no face detected, stop
      self.twist.angular.z = 0
      self.twist.linear.x = 0
      self.pub.publish(self.twist)


    # print self.twist
    if cv2.waitKey(1) & 0xFF == ord('q'):
      # ready=True 
      cv2.waitKey(1)
      return False
    return True
      # running = False


p = Predictor()
while p.run():
  pass