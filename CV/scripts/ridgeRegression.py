import numpy as np
from sklearn import linear_model
from os import listdir
from os.path import isfile, join
import cv2



def get_training_data():
  """Collect data from compressed images and convert to X_training and Y_training
  inputs: none
  outputs: X_train - training set for x values
           Y_train - training set of y values """
  with open('timeStamptoName.txt','r') as inf:
    timeStampToName = eval(inf.read())
  X_train = []
  Y_train_direction = []
  Y_train_name = []
  width = 800 #width of screen of pygame ball
  frameRate = 50 # rate of pictures which are taken by number of pixels
  i = 0
  while i <= width:
    directory = '../images/400_' + str(i)
    print 'a', i
    onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    for item in onlyfiles:
      im = cv2.imread(directory+'/'+item, 0)
      if im.shape == (24, 24):
        reshaped = np.reshape(im, 576)
        X_train.append(reshaped)
        Y_train_direction.append((400,i))
        Y_train_name.append(timeStampToName[item])

    directory = '../images/' + str(i) + '_400'
    print 'b', i
    onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    for item in onlyfiles:
      im = cv2.imread(directory+'/'+item, 0)
      if im.shape == (24, 24):
        reshaped = np.reshape(im, 576)
        X_train.append(reshaped)
        Y_train_direction.append((i, 400))
        Y_train_name.append(timeStampToName[item])


    i+= frameRate
  print Y_train_name

  return (X_train, Y_train_direction, Y_train_name)

def ridge_predict(X_test, ridge):
  """predicts a pitch and yaw from an input image
  inputs: X_test - 24 by 24 image file of a face
          ridge - ridge prediction model calibrated for this training set
  outputs: yaw - predicted yaw of input image
           pitch - predicted pitch of input image"""

  prediction =  ridge.predict(X_test)
  return prediction[0]

def get_ridge_model(alpha):
  """ creates a ridge model from the files saved in the directory folders
  inputs: alpha - parameter in ridge model for controling precision
  outputs: ridge - scikit learn ridge model"""

  X_train, Y_train = get_training_data()
  ridge = linear_model.Ridge(alpha=alpha)
  ridge.fit(X_train, Y_train)
  return ridge

if __name__ == '__main__':
  get_training_data()