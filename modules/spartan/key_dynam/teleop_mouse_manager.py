import sys
import pygame
from pygame.locals import *


class TeleopMouseManager():

    def __init__(self):

        pygame.init()
        screen_size = 1
        self.screen = pygame.display.set_mode((screen_size, screen_size))

        self.grab_mouse_focus()

        self.side_button_back_DOWN = False
        self.side_button_fwd_DOWN = False

    def grab_mouse_focus(self):
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)

    def release_mouse_focus(self):
        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)

    def get_events(self):
        events = dict()
        events["escape"] = False
        events["keydown"] = set()

        mouse_wheel_up = mouse_wheel_down = side_button_back = side_button_forward = False

        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit(0)
            elif event.type == KEYDOWN:
                events['keydown'].add(event.key)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                events["escape"] = True
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



        if K_RETURN in events['keydown']:
            self.grab_mouse_focus()

        if K_SPACE in events['keydown'] or K_ESCAPE in events['keydown']:
            self.release_mouse_focus()

        # these are zero if you don't have the mouse focus
        delta_x, delta_y = pygame.mouse.get_rel()

        rotate_left, _, rotate_right = pygame.mouse.get_pressed()

        events["delta_x"] = delta_x
        events["delta_y"] = delta_y
        events["mouse_wheel_up"] = mouse_wheel_up
        events["mouse_wheel_down"] = mouse_wheel_down
        events["rotate_left"] = rotate_left
        events["rotate_right"] = rotate_right
        events["side_button_back"] = self.side_button_back_DOWN
        events["side_button_forward"] = self.side_button_fwd_DOWN

        pygame.event.clear()

        return events