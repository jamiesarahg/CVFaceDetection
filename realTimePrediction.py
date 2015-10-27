import cv2
import followDot
import ridgeRegression
import numpy as np

def aveList(listInput):
  return sum(listInput)/len(listInput)
  
def sendToNeato(pub, twist, currentYaw, currentPitch):
  twist.angular.z = (currentYaw-400) * .001
  twist.linear.x = (currentPitch-400) * .001
  pub.publish(self.twist)

if __name__ == "__main__":
  ridge = ridgeRegression.get_ridge_model(.1)
  camera = followDot.initializeCamera()
  face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')
  running = True
  previous12values =  ([0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0])

  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  twist = Twist()


  while running == True:
    ret, frame = camera.read()
    faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
    for (x,y,w,h) in faces:
      #draws rectangle on face
      cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))
    try:
      cropped = followDot.cropImage(frame, faces[0])
      compressed = followDot.compressImage(cropped)
      compressed = np.asarray(compressed, dtype=np.uint8)
      gray = cv2.cvtColor(compressed, cv2.COLOR_BGR2GRAY )
      reshaped = np.reshape(gray, 576)
      yaw, pitch =  ridgeRegression.ridge_predict([reshaped],ridge)
      del previous12values[0][0]
      del previous12values[1][0]
      previous12values[0].append(yaw)
      previous12values[1].append(pitch)
      currentYaw = aveList(previous12values[0])
      currentPitch = aveList(previous12values[1])
      print 'yaw', currentYaw, 'pitch', currentPitch

      sentToNeato(pub, twist, currentYaw, currentPitch)


    except IndexError: 
      pass
    cv2.imshow('frame',frame)
    #waits until user presses q to start
    if cv2.waitKey(1) & 0xFF == ord('q'):
      ready=True 
      cv2.waitKey(1)
      running = False
