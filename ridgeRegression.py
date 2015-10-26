import numpy as np
from sklearn.linear_model import Ridge

def get_training_data():
  """Collect data from compressed images and convert to X_training and Y_training
  inputs: none
  outputs: X_train - training set for x values
           Y_train - training set of y values """

  return (X_train, Y_train)

def ridge_predict(X_test, ridge):
  """predicts a pitch and yaw from an input image
  inputs: X_test - 24 by 24 image file of a face
          ridge - ridge prediction model calibrated for this training set
  outputs: yaw - predicted yaw of input image
           pitch - predicted pitch of input image"""

  yaw, pitch = ridge.predict(X_test)

if __name__ == "__main__":

  X_train, Y_train = get_training_data()
  ridge = linear_model.Ridge(alpha=.1)
  ridge.fit(X_train, Y_train)