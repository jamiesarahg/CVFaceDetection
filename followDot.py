import pygame
import time
import cv2
import os
import numpy as np

def mkdirs(width):
	try:
		os.mkdir('images')
	except OSError:
		pass

	i = 0
	while i <=width:
		try:
			os.mkdir('images/{0}_400'.format(i))
			os.mkdir('images/400_{0}'.format(i))
		except OSError:
			pass
		i += frameRate


def compressImage(im, _file):
	# im =  cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
	squareSize = im.shape[0]/24
	posY = 0
	newIm = np.ndarray((24, 24, 3))
	while posY < im.shape[1]:
		posX = 0
		while posX < im.shape[0]: 	# go across
			a = im[posX: posX + squareSize, posY: posY + squareSize]
			sumB, sumG, sumR = [0, 0, 0]
			for i in range(squareSize):
				for j in range(squareSize):
					sumB += a[i][j][0]
					sumG += a[i][j][1]
					sumR += a[i][j][2]
			sumB = sumB/(squareSize)**2
			sumG = sumG/(squareSize)**2
			sumR = sumR/(squareSize)**2
			newIm[posX/squareSize][posY/squareSize] = [sumB, sumG, sumR]
			posX += squareSize
		posY += squareSize

	cv2.imwrite(_file, newIm)

def getImage(camera, name):
	face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')
	retval, im = camera.read()
	faces = face_cascade.detectMultiScale(im, scaleFactor=1.2, minSize=(20,20))
	if faces!= ():
		(x,y,w,h) = faces[0]
		#crop images such that the dimensions are a multiple of 24
		if h%24 >0:
			try:
 				im = im[y:y + h + 24 - h%24, x:x + w + 24 - w%24]
	 		except IndexError:
	 			im = im[y:y + h - h%24, x:x + w - w%24]
	 	else:
	 		im = im[y:y + h, x:x + w]
		_file = "images/" + name + ".png"
 		cv2.imwrite(_file, im)

def initializeCamera():
	camera_port = 0
	camera = cv2.VideoCapture(camera_port) 
	for i in xrange(10):
		retval, im = camera.read()

	return camera

def initializeScreen(width, height):
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
	white = (255,255,255)
	black = (0,0,0)
	sleepTime = .001

	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	time.sleep(sleepTime)

def intakeData(camera, screen, width, height, frameRate):
	timestamp = time.time()
	x=width/2
	y=height/2

	while x < width:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1

	while x >0:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x-=1



	while x<width/2:
		updateScreen(x, y, screen)
		if x%frameRate == 0:
			getImage(camera,'{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1


	while y < height:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1


	while y >0:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y-=1


	while y<height/2:
		updateScreen(x, y, screen)
		if y%frameRate == 0:
			getImage(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1

	
	i = 0
	while i <=width:
		_file = 'images/400_{0}/{1}.png'.format(i, timestamp)
		im = cv2.imread(_file)
		if type(im) == np.ndarray:
			print type(im)
			compressImage(im, _file)
		
		_file = 'images/{0}_400/{1}.png'.format(i, timestamp)
		im = cv2.imread(_file)
		if type(im) == np.ndarray:
			print type(im)
			compressImage(im, _file)
		
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
			cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))
		cv2.imshow('frame',frame)
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
