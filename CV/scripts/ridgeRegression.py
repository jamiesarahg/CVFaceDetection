import numpy as np
from sklearn import linear_model
from os import listdir
from os.path import isfile, join
import cv2
import face_recognition
from sklearn.decomposition import RandomizedPCA
from sklearn.grid_search import GridSearchCV
from sklearn.svm import SVC




def get_training_data():
  """Collect data from compressed images and convert to X_training and Y_training
  inputs: none
  outputs: X_train - training set for x values
           Y_train - training set of y values """
  with open('timeStamptoName.txt','r') as inf:
    timeStampToName = eval(inf.read())
  with open('nameToNumber.txt','r') as inf:
    numberToName = eval(inf.read())
  X_train = []
  Y_train_direction = []
  Y_train_name = []
  width = 800 #width of screen of pygame ball
  frameRate = 50 # rate of pictures which are taken by number of pixels
  i = 0
  while i <= width:
    
    directory = '../images/400_' + str(i)
    onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    for item in onlyfiles:
      im = cv2.imread(directory+'/'+item, 0)
      if im.shape == (24, 24):
        reshaped = np.reshape(im, 576)
        X_train.append(reshaped)
        Y_train_direction.append((400,i))
        name = timeStampToName[item]
        for k, v in numberToName.items():
          if v == name:
            Y_train_name.append(k)
            break

    directory = '../images/' + str(i) + '_400'
    print 'b', i
    onlyfiles = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    for item in onlyfiles:
      im = cv2.imread(directory+'/'+item, 0)
      if im.shape == (24, 24):
        reshaped = np.reshape(im, 576)
        X_train.append(reshaped)
        Y_train_direction.append((i, 400))
        name = timeStampToName[item]
        for k, v in numberToName.items():
          if v == name:
            Y_train_name.append(k)
            break
    i+= frameRate
  return (X_train, Y_train_direction, Y_train_name)

def ridge_predict(X_test, ridge):
  """predicts a pitch and yaw from an input image
  inputs: X_test - 24 by 24 image file of a face
          ridge - ridge prediction model calibrated for this training set
  outputs: yaw - predicted yaw of input image
           pitch - predicted pitch of input image"""

  prediction =  ridge.predict(X_test)
  return prediction[0]

def get_ridge_model(X_train, Y_train_direction, alpha):
  """ creates a ridge model from the files saved in the directory folders
  inputs: alpha - parameter in ridge model for controling precision
  outputs: ridge - scikit learn ridge model"""

  ridge = linear_model.Ridge(alpha=alpha)
  ridge.fit(X_train, Y_train_direction)
  return ridge

def get_face_recognition_model(X_train, Y_train):
  """ does a principle component analysis of all of the faces, then gets the
  eigenfaces, then runs a support vector machine that fits the compressed data
  inputs: X_train traning set. list of images
          Y_train - supervised targets, list of integers representing individual faces
  outputs: clf - SVC classifier
            pca - PCA algorithm"""

  n_components = 100 # number of components of PCA

  #run a PCA on the training data
  pca = RandomizedPCA(n_components=n_components, whiten=True).fit(X_train)
  #dimensions of the incoming images
  h = 576
  w = 1
  X_train = np.array(X_train) #turning list inton numpy array

  #reshaping pca components into eigenfaces if you want to view them
  eigenfaces = pca.components_.reshape((n_components, h, w))
  #running PCA on training set
  X_train_pca = pca.transform(X_train)

  # establishing parameters for SVM:
  # C - misclassification vs simplicity
  # gamma - influence of one single traning example
  param_grid = {'C': [1e3, 5e3, 1e4, 5e4, 1e5],
                'gamma': [0.0001, 0.0005, 0.001, 0.005, 0.01, 0.1], }

  # param_grid = {'C': [1e3],#[1e3, 5e3, 1e4, 5e4, 1e5],
  #               'gamma': [.0001],}

  print("Fitting the classifier to the training set")
  clf = GridSearchCV(SVC(kernel='rbf', class_weight='auto'), param_grid)
  clf = clf.fit(X_train_pca, Y_train)
  return clf, pca

def get_models():
  """ Overarching function to build model for both face recognition and face yaw and pitch
      inputs: none
      outputs: ridge - model for determining yaw and pitch of a face
                clf - model for recognizing a face
                pca - principle components used for facial recognition"""
    # collects data from files
  X_train, Y_train_direction, Y_train_name = get_training_data()
  #runs face angle model
  ridge = get_ridge_model(X_train, Y_train_direction, .1)
  #runs face recognition model
  clf, pca = get_face_recognition_model(X_train, Y_train_name)
  return ridge, clf, pca





if __name__ == '__main__':
  X_train, Y_train_direction, Y_train_name = get_training_data()
  face_recognition.PCA(X_train, Y_train_name, 100)