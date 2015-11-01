import numpy as np
from sklearn import linear_model
# from ImageManipulation import ImageManipulation
from DataCollection import DirManager
# import os
# from os import listdir
# from os.path import isfile, join
import cv2


class RidgeModel(DirManager):
  def __init__(alpha=.1):
    self.X_train, self.Y_train = self.getTrainingData()
    self.ridge = linear_model.Ridge(alpha=alpha)
    self.ridge.fit(X_train, Y_train)

  def getTrainingData(self):
    """Collect data from compressed images and convert to X_training and Y_training
    inputs: none
    outputs: X_train - training set for x values
             Y_train - training set of y values """
    X_train = []
    Y_train = []
    for (_file, x, y) in self.getDirFiles():
      im = cv2.imread(_file, 0)
      if im.shape == (24, 24):
        reshaped = np.reshape(im, 576)
        X_train.append(reshaped)
        Y_train.append((x,y))

    return (X_train, Y_train)

  # def ridge_predict(self, X_test):
  #   """predicts a pitch and yaw from an input image
  #   inputs: X_test - 24 by 24 image file of a face
  #           ridge - ridge prediction model calibrated for this training set
  #   outputs: yaw - predicted yaw of input image
  #            pitch - predicted pitch of input image"""

  #   prediction =  ridge.predict(X_test)
  #   return prediction[0]

  # def get_ridge_model(alpha):
  #   """ creates a ridge model from the files saved in the directory folders
  #   inputs: alpha - parameter in ridge model for controling precision
  #   outputs: ridge - scikit learn ridge model"""

  #   X_train, Y_train = get_training_data()
  #   ridge = linear_model.Ridge(alpha=alpha)
  #   ridge.fit(X_train, Y_train)
  #   return ridge