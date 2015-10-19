import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

cap = cv2.VideoCapture(0)
while(True): 
	# Capture frame-by-frame 
	ret, frame = cap.read()

	faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
	for (x,y,w,h) in faces:
		cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255)) 

	# Display the resulting frame 
	cv2.imshow('frame',frame) 
	if cv2.waitKey(1) & 0xFF == ord('q'): 
		break 

# When everything done, release the capture 
cap.release() 
cv2.destroyAllWindows()