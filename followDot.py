from visual import *
import time
 
time.sleep(1)
scene2 = display(title='Follow the Ball',
     x=0, y=0, width=800, height=800,
     center=(300,300,0))
ball = sphere(pos=(301,301,0), radius = 10, color=color.white)
ball.velocity = vector(0,0,0)
deltat = 0/005
t=0
ball.pos = ball.pos + ball.velocity*deltat
 