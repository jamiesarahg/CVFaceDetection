import pygame
import time

pygame.init()
screen = pygame.display.set_mode((640,640))
pygame.draw.circle(screen, (255,255,255), (320,320), 10)
pygame.display.update()
time.sleep(10)
