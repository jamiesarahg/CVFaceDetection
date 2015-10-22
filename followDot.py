import pygame
import time
import cv2
import os

def mkdirs(width):
	i = 0
	while i <=width:
		try:
			os.mkdir('images/{0}_400'.format(i))
			os.mkdir('images/400_{0}'.format(i))
		except OSError:
			pass
		i += 50


def get_image(camera, name):
	retval, im = camera.read()
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




width = 800
height = 800
mkdirs(width)



camera = initializeCamera()
screen = initializeScreen(width, height)

for i in range(3):
	timestamp = time.time()
	x=width/2
	y=height/2

	while x < width:
		updateScreen(x, y, screen)
		if x%50 == 0:
			get_image(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1

	while x >0:
		updateScreen(x, y, screen)
		if x%50 == 0:
			get_image(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		x-=1



	while x<width/2:
		updateScreen(x, y, screen)
		if x%50 == 0:
			get_image(camera,'{0}_{1}/{2}'.format(x, y, timestamp))
		x+=1


	while y < height:
		updateScreen(x, y, screen)
		if y%50 == 0:
			get_image(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1


	while y >0:
		updateScreen(x, y, screen)
		if y%50 == 0:
			get_image(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y-=1


	while y<height/2:
		updateScreen(x, y, screen)
		if y%50 == 0:
			get_image(camera, '{0}_{1}/{2}'.format(x, y, timestamp))
		y+=1

del(camera)