import time
import sys
import pygame
from pygame.locals import *
pygame.init()

screen_size = 1
screen = pygame.display.set_mode((screen_size, screen_size))
clock = pygame.time.Clock()

carrot_wait = 0

pygame.event.set_grab(True)
pygame.mouse.set_visible(False)

HZ = 100.0

while True:

    mouse_wheel_up, mouse_wheel_down = False, False

    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit(0)
        elif event.type == KEYDOWN and event.key == K_ESCAPE:
            sys.exit(0)
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:   
                  mouse_wheel_up = True
            if event.button == 5:   
                  mouse_wheel_down = True

    keys = pygame.key.get_pressed()
    
    delta_x, delta_y = pygame.mouse.get_rel()
    print delta_x, delta_y
    mouse_force_scale = 1000

    rotate_left, _, rotate_right = pygame.mouse.get_pressed()

    print mouse_wheel_down, mouse_wheel_up
    time.sleep(1/(HZ))