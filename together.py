import numpy as np
import cv2
import pygame
import time

# pygame color values
white = (255,255,255)
black = (0,0,0)

# establishing width and height for pygame screen
width = 800
height = 800

# establishing starting position for the ball
x_ball = width/2
y_ball = height/2 

#initiating steps for ball
a = True
b, c, d, e, f, g = False, False, False, False, False, False

# initiating pygame
pygame.init()
screen = pygame.display.set_mode((width,height)) #creating screen
pygame.draw.circle(screen, white, (x_ball,y_ball), 5) #drawing ball
pygame.display.update()

#classifier for face detector
face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

#starts video capture
cap = cv2.VideoCapture(0)


# Init video
ret, frame = cap.read()

faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
for (x,y,w,h) in faces:
	cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255)) 

# Display the resulting frame 
cv2.imshow('frame',frame) 


while(True): 

	# update ball location
	screen.fill(black)
	pygame.draw.circle(screen, white, (x_ball,y_ball), 5)
	pygame.display.update()

	# Capture frame-by-frame 
	ret, frame = cap.read()

	# Face detecting and drawing rectangle around face
	faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
	for (x,y,w,h) in faces:
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255)) 

	# Display the resulting frame 
	cv2.imshow('frame',frame) 
	if cv2.waitKey(1) & 0xFF == ord('q'): 
		break 

	# move ball for next round
	if a:
		if x_ball < width:
			x_ball+= 1
		else:
			a =False
			b = True
	if b:
		if x_ball>0:
			x_ball-=1
		else:
			b = False
			c = True
	if c:
		if x_ball < width/2:
			x_ball+=1
		else:
			c= False
			d = True
	if d:
		if y_ball < height:
			y_ball+= 1
		else:
			d =False
			e = True
	if e:
		if y_ball>0:
			y_ball-=1
		else:
			d = False
			f = True
	if g:
		if y_ball < height/2:
			y_ball+=1
		else:
			g= False
			a = True





# When everything done, release the capture 
cap.release() 
cv2.destroyAllWindows()