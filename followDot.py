import pygame
import time
import cv2
import os
import numpy as np

def mkdirs(width, frameRate):
	""" if directories to collect images do not exist, make the respective directories
			input: width of screen of pong ball
						 frameRate = number of pixels of dot on screen between which we take pictures
			output: none, but creates directories in local storage	"""
	try:
		os.mkdir('images')
	except OSError:
		pass

	i = 0
	while i <=width:
		#create folders for each yaw and pitch of the face
		try:
			os.mkdir('images/{0}_400'.format(i))
			os.mkdir('images/400_{0}'.format(i))
		except OSError:
			pass
		i += frameRate


def compressImage(im):
	"""compresses Images to 24 by 24 pixel images
	input: im - image intakeData
					_file - filename
	output: compressed image """

	squareSize = im.shape[0]/24 # number of pixels from one direction that will be compressed into one pixel
	posY = 0
	newIm = np.ndarray((24, 24, 3)) # creating array for compressed image
	while posY < im.shape[1]:
		posX = 0
		while posX < im.shape[0]: 	# go across
			a = im[posX: posX + squareSize, posY: posY + squareSize] #crop image to square that will be compressed to one pixel
			#sum the BGR values across the square to make an average
			sumB, sumG, sumR = [0, 0, 0]
			for i in range(squareSize):
				for j in range(squareSize):
					sumB += a[i][j][0]
					sumG += a[i][j][1]
					sumR += a[i][j][2]
			sumB = sumB/(squareSize)**2
			sumG = sumG/(squareSize)**2
			sumR = sumR/(squareSize)**2

			#recompile the new image
			newIm[posX/squareSize][posY/squareSize] = [sumB, sumG, sumR]

			#increment to the next square for compression
			posX += squareSize
		posY += squareSize
	return newIm

	

def getImage(camera, name):
	"""takes in image from camera, identifies face in the image and saves the image
	inputs: camera - data from camera
					name - filename
	output: none"""

	face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')
	retval, im = camera.read()
	faces = face_cascade.detectMultiScale(im, scaleFactor=1.2, minSize=(20,20))
	if faces!= ():
	 	im = cropImage(im,faces[0])
	# 	(x,y,w,h) = faces[0]
	# 	#crop images such that the pixel dimensions are squares with sides of multiple of 24
	# 	if h%24 >0:
	# 		try:
 # 				im = im[y:y + h + 24 - h%24, x:x + w + 24 - w%24]
	#  		except IndexError:
	#  			im = im[y:y + h - h%24, x:x + w - w%24]
	#  	else:
	#  		im = im[y:y + h, x:x + w]



	 	#save images
		_file = "images/" + name + ".png"
 		cv2.imwrite(_file, im)

def cropImage(im, faceData):
	(x,y,w,h) = faceData
	#crop images such that the pixel dimensions are squares with sides of multiple of 24
	if h%24 >0:
		try:
				im = im[y:y + h + 24 - h%24, x:x + w + 24 - w%24]
 		except IndexError:
 			im = im[y:y + h - h%24, x:x + w - w%24]
 	else:
 		im = im[y:y + h, x:x + w]
 	return im


def initializeCamera():
	""" helper function to initialize camera
	input - none
	output - camera data"""

	camera_port = 0
	camera = cv2.VideoCapture(camera_port) 

	#gets rid of first 10 seconds so that camera focuses
	for i in xrange(10):
		retval, im = camera.read()

	return camera

def initializeScreen(width, height):
	""" starts screen for ball to be monitored on
			input: width - width of screen
						 height - height of screen
			output: pygame screen """

	white = (255,255,255)
	black = (0,0,0)
	sleepTime = .005
	pygame.init()
	screen = pygame.display.set_mode((width,height))
	pygame.draw.circle(screen, white, (width/2,height/2), 5)
	pygame.display.update()
	time.sleep(sleepTime)
	return screen

def updateScreen(x, y, screen):
	"""updates location of ball on screen
			inputs: x, y  - position of ball
							screen - pygame screen 
			outputs: none
			redisplays screen """

	white = (255,255,255)
	black = (0,0,0)
	sleepTime = .001

	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	time.sleep(sleepTime)

def intakeData(camera, screen, width, height, frameRate):
	""" run function for intaking data
			inputs: camera - camera data
							screen - pygame screen data
							width - width of pygame screen
							height - height of pygame screen
							frameRate - number of pixels that ball will move between image save
			outputs: none, but images are saved to local storage
			"""

	timestamp = time.time()
	x=width/2
	y=height/2

	# moves ball to right of screen
	while x < width:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1
	# moves ball to left of screen
	while x >0:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x-=1

	#moves ball to center of screen
	while x<width/2:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera,'{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1

	#moves ball to top of screen
	while y < height:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1

	# moves ball to bottom of screen
	while y >0:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y-=1

		# moves ball to center of screen
	while y<height/2:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1

	#compressess all of the images to 24 by 24 pixel square images and saves them to local storage
	i = 0
	while i <=width:
		_file = 'images/400_{0}/{1}.png'.format(i, timestamp)
		im = cv2.imread(_file)
		#only compress if image exists
		if type(im) == np.ndarray:
			print type(im)
			newIm = compressImage(im)
			cv2.imwrite(_file, newIm)
		
		_file = 'images/{0}_400/{1}.png'.format(i, timestamp)
		im = cv2.imread(_file)
		#only compress if image exists
		if type(im) == np.ndarray:
			print type(im)
			newIm = compressImage(im)
			cv2.imwrite(_file, newIm)
		
		i += frameRate


if __name__ == "__main__":

	width = 800
	height = 800
	frameRate = 50

	ready = False
	mkdirs(width)

	camera = initializeCamera()
	screen = initializeScreen(width, height)
	face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')


	while ready == False:
		ret, frame = camera.read()
		faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
		for (x,y,w,h) in faces:
			#draws rectangle on face
			cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))
		cv2.imshow('frame',frame)
		#waits until user presses q to start
		if cv2.waitKey(1) & 0xFF == ord('q'):
			ready=True 
			cv2.waitKey(1)
			cv2.destroyWindow('frame')
			cv2.destroyAllWindows()
			cv2.waitKey(1)
			break
	time.sleep(1)

	intakeData(camera, screen, width, height, frameRate)

	del(camera)
