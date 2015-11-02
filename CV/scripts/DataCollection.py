import os
import pygame
import time
import cv2
import numpy as np
from ImageManipulation import ImageManipulation

class DirManager(object):
  def __init__(self, width=-1, frameRate=0):
    self.width = width
    self.frameRate = frameRate

  def mkdirs(self):
    """ creates directories in local storage if they don't already exist
        input: width - screen of pong ball width
               frameRate - how many pixels between each photo
        returns: none """
    try: #make an images directory
      os.mkdir('images')
    except OSError: #if it already exists
      pass

    i = 0
    while i <=self.width:
      #create folders for each yaw and pitch of the face
      try:
        os.mkdir('images/{0}_400'.format(i))
        os.mkdir('images/400_{0}'.format(i))
      except OSError:
        pass
      i += self.frameRate

  def getDirFiles(self):
    '''
    inputs: none
    returns: list of tuples (full file path, x coordinate, y coordinate)
    '''
    files = []
    parentDir = os.getcwd() + '/images'
    for dirName in os.listdir(parentDir):
      directory = os.path.join(parentDir, dirName)
      x, y = dirName.split('_')
      for _file in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, _file)):
          files.append((os.path.join(directory, _file), x, y))
    return files

class DataCollection(DirManager):
  def __init__(self):
    self.width = 800
    self.height = 800
    self.frameRate = 50
    self.mkdirs()
    self.ImManipulator = ImageManipulation()
    self.ImManipulator.camera.showVideoOnStartUp()
    self.screen  = self.initializeScreen()
    self.intakeData()
    del(self.ImManipulator.camera.cam)
    self.compressAll()


  def initializeScreen(self):
    """ starts screen for ball to be monitored on
        input: width - width of screen
               height - height of screen
        output: pygame screen """

    white = (255,255,255)
    black = (0,0,0)
    sleepTime = .005 
    pygame.init()
    screen = pygame.display.set_mode((self.width,self.height))
    pygame.draw.circle(screen, white, (self.width/2,self.height/2), 5)
    pygame.display.update()
    time.sleep(sleepTime)
    return screen

  def updateScreen(self, x, y):
    """updates location of ball on screen
        inputs: x, y  - position of ball
                screen - pygame screen 
        outputs: none """

    white = (255,255,255)
    black = (0,0,0)
    sleepTime = .001

    self.screen.fill(black)
    pygame.draw.circle(self.screen, white, (x,y), 5)
    pygame.display.update()
    time.sleep(sleepTime)

  def intakeData(self):
    """ run function for intaking data
        inputs: camera - camera data
                width - width of pygame screen
                height - height of pygame screen
                frameRate - number of pixels that ball will move between image save
        outputs: returns none, images get saved to local storage
        """

    self.timestamp = time.time()
    x=self.width/2
    y=self.height/2

    # moves ball to right of screen
    while x < self.width:
      self.updateScreen(x, y)
      if x%self.frameRate == 0:
        self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      x+=1
    
    # moves ball to left of screen
    while x >0:
      self.updateScreen(x, y)
      if x%self.frameRate == 0:
         self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      x-=1

    #moves ball to center of screen
    while x<self.width/2:
      self.updateScreen(x, y)
      if x%self.frameRate == 0:
        self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      x+=1

    #moves ball to top of screen
    while y < self.height:
      self.updateScreen(x, y)
      if y%self.frameRate == 0:
        self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      y+=1

    # moves ball to bottom of screen
    while y >0:
      self.updateScreen(x, y)
      if y%self.frameRate == 0:
        self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      y-=1

    # moves ball to center of screen
    while y<self.height/2:
      self.updateScreen(x, y)
      if y%self.frameRate == 0:
        self.ImManipulator.getCameraImage('{0}_{1}/{2}'.format(x, y, self.timestamp))
      y+=1

  #compressess all of the images saves over them in local storage
  def compressAll(self):
    for (_file, _, _) in self.getDirFiles():
      im = cv2.imread(_file)
      if type(im) == np.ndarray:
        newIm = self.ImManipulator.compressImage(im)
        cv2.imwrite(_file, newIm)




if __name__ == '__main__':
  DataCollection()