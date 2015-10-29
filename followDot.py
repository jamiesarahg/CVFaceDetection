import pygame
import time
import cv2
import os
import numpy as np

def mkdirs(width, frameRate):
	""" creates directories in local storage if they don't already exist
			input: width - screen of pong ball width
						 frameRate - how many pixels between each photo
			returns: none """
	try: #make an images directory
		os.mkdir('images')
	except OSError: #if it already exists
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

	squareSize = im.shape[0]/24 # number of pixels on 1 side of the square that will become an averaged pixel
	posY = 0
	newIm = np.ndarray((24, 24, 3)) # creating array for compressed image
	while posY < im.shape[1]:
		posX = 0
		while posX < im.shape[0]: 	# go across
			a = im[posX: posX + squareSize, posY: posY + squareSize] #crop image to one square
			#sum the BGR values across the square
			sumB, sumG, sumR = [0, 0, 0]
			for i in range(squareSize):
				for j in range(squareSize):
					try:
						sumB += a[i][j][0]
						sumG += a[i][j][1]
						sumR += a[i][j][2]
					except IndexError: #we were getting sporadic errors here
						print squareSize
						print im.shape
						
			sumB = sumB/(squareSize)**2 #get the average BGR values
			sumG = sumG/(squareSize)**2
			sumR = sumR/(squareSize)**2

			#update one pixel in the compressed image array
			try:
			 newIm[posX/squareSize][posY/squareSize] = [sumB, sumG, sumR]
			except IndexError: #and also here
				print squareSize
				print posX
				print posX/squareSize
				print posY/squareSize
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
	retval, im = camera.read() #take webcame photo
	faces = face_cascade.detectMultiScale(im, scaleFactor=1.2, minSize=(20,20)) #detect faces
	if faces!= (): #if faces are found
		im = cropImage(im, faces)

	 	#save images
		_file = "images/" + name + ".png"
		cv2.imwrite(_file, im)

def cropImage(im, faces):
	"""crops image to include only the face
	input: 	im - the full image to crop
					faces - list of detected faces in image
	output:	cropped image """

	#choose largest detected face
	maxIndex, maxSize = [0, 0];
	for i, f in enumerate(faces):
		size = (f[2])*(f[3])
		if size > maxSize:
			maxIndex = i
			maxSize = size
	
	faceData = faces[maxIndex]
	(x,y,w,h) = faceData

	#crop such that image dimensions are multiples of 24
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
	output - camera object"""

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
			outputs: none """

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
			outputs: returns none, images get saved to local storage
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

	#compressess all of the images saves over them in local storage
	for directory in os.listdir(os.getcwd()+ '/images'):
		_file = os.getcwd() + '/images/' + directory + '/' + str(timestamp) + '.png'
		im = cv2.imread(_file)
		if type(im) == np.ndarray:
			newIm = compressImage(im, _file)
			cv2.imwrite(_file, newIm)

if __name__ == "__main__":

	width = 800
	height = 800
	frameRate = 50

	ready = False
	mkdirs(width, frameRate) #make directory structure

	camera = initializeCamera() 
	screen = initializeScreen(width, height)
	face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

	#before experiment is ready to start
	while ready == False: #show video and mark detected faces
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
