import sys
import pygame
from pygame.locals import *


class TeleopMouseManager():

    def __init__(self):

        pygame.init()
        screen_size = 1
        self.screen = pygame.display.set_mode((screen_size, screen_size))

        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

        self.side_button_back_DOWN = False
        self.side_button_fwd_DOWN = False

    def get_mouse_events(self):

        mouse_wheel_up = mouse_wheel_down = side_button_back = side_button_forward = False

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
                if event.button == 8:
                    self.side_button_back_DOWN = True
                if event.button == 9:
                    self.side_button_fwd_DOWN = True
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 8:
                    self.side_button_back_DOWN = False
                if event.button == 9:
                    self.side_button_fwd_DOWN = False

        keys = pygame.key.get_pressed()
        
        w = a = s = d = False
        if keys[K_w]:
            w = True
        if keys[K_a]:
            a = True
        if keys[K_s]:
            s = True
        if keys[K_d]:
            d = True
            
        
        delta_x, delta_y = pygame.mouse.get_rel()
        rotate_left, _, rotate_right = pygame.mouse.get_pressed()

        mouse_events = dict()
        mouse_events["delta_x"] = delta_x
        mouse_events["delta_y"] = delta_y
        mouse_events["w"] = w
        mouse_events["a"] = a
        mouse_events["s"] = s
        mouse_events["d"] = d
        mouse_events["mouse_wheel_up"] = mouse_wheel_up
        mouse_events["mouse_wheel_down"] = mouse_wheel_down
        mouse_events["rotate_left"] = rotate_left
        mouse_events["rotate_right"] = rotate_right
        mouse_events["side_button_back"] = self.side_button_back_DOWN
        mouse_events["side_button_forward"] = self.side_button_fwd_DOWN
        return mouse_events