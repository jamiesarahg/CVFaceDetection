import pygame
import time

white = (255,255,255)
black = (0,0,0)
width = 800
height = 800
sleepTime = .005

x=width/2
y=height/2

pygame.init()
screen = pygame.display.set_mode((width,height))
pygame.draw.circle(screen, white, (x,y), 5)
pygame.display.update()
time.sleep(sleepTime)



while x < width:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	x+=1
	time.sleep(sleepTime)

while x >0:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	x-=1
	time.sleep(sleepTime)

while x<width/2:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	x+=1
	time.sleep(sleepTime)

while y < height:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	y+=1
	time.sleep(sleepTime)

while y >0:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	y-=1
	time.sleep(sleepTime)

while y<height/2:
	screen.fill(black)
	pygame.draw.circle(screen, white, (x,y), 5)
	pygame.display.update()
	y+=1
	time.sleep(sleepTime)
