
import asyncio
import serial_asyncio
import asyncio_mqtt as mqtt
try:
    from contextlib import AsyncExitStack, asynccontextmanager
except:
    from async_exit_stack import AsyncExitStack


import numpy as np
from scipy.spatial import transform
import math
import re

import json
import random
import yaml

from sys import argv
import os
import time
import simpleaudio as sa
import subprocess
# import skimage.io
# import skimage.draw
import imageio

import numpy as np

from scipy.spatial.distance import euclidean
from scipy.ndimage import median_filter
import time
from scipy.optimize import minimize
import board
import adafruit_bno055
from contextlib import contextmanager

from inspect import isawaitable
import csv

from putzini_config import PutziniConfig


class KeepoutError(Exception):

    def __init__(self, message='', current_pos=None, target_pos=None, mqtt_client=None):
        self.current_pos = current_pos
        self.target_pos = target_pos
        self.message = 'Keepout error ' if not message else message
        if current_pos is not None:
            self.message += f'at {tuple(round(c*100,1) for c in current_pos)} cm'
        if target_pos is not None:
            self.message += f' going to {tuple(round(c*100,1) for c in target_pos)} cm'
        if mqtt_client is not None:
            asyncio.ensure_future(mqtt_client.publish("putzini/error", self.message))
        super().__init__(self.message)

class PutziniKeepoutArea:

    def __init__(self, mqtt_client, putzini_config: PutziniConfig, 
        putzini_drive=None, putzini_state=None):
        # the image should have 1px per mm
        img = imageio.imread(putzini_config.keepout_img)
        self.state = putzini_state
        self.keepout_map = (img == 0)
        self.forbid_map = (img != 255)
        self.ref = self.keepout_map.shape[0]/2, self.keepout_map.shape[1]/2
        self.drive = putzini_drive
        self.fac = 100 # set to 100 if parameters are supposed to be meters
        self.stop = True
        self.mqtt_client = mqtt_client
        
    def is_point_keepout(self, x, y):
        Y, X = int(y*self.fac+self.ref[0]), int(x*self.fac+self.ref[1])
        if (X < 0) or (X >= self.keepout_map.shape[1]) or (Y < 0) or (Y >= self.keepout_map.shape[1]):
            return True

        return self.keepout_map[Y, X] > 0
        
    def is_point_forbidden(self, x, y):
        Y, X = int(y*self.fac+self.ref[0]), int(x*self.fac+self.ref[1])
        if (X < 0) or (X >= self.forbid_map.shape[1]) or (Y < 0) or (Y >= self.forbid_map.shape[1]):
            return True

        return self.forbid_map[Y, X] > 0

    def N_line_keepout(self, x1, y1, x2, y2):
        N_pts = int(np.ceil(((self.fac*x2-self.fac*x1)**2 + (self.fac*y2-self.fac*y1)**2)**.5))
        x, y = np.linspace(x1, x2, N_pts), np.linspace(y1, y2, N_pts)
        try:
            return np.sum(self.keepout_map[(y*self.fac+self.ref[0]).astype(int), (x*self.fac+self.ref[1]).astype(int)])/self.fac
        except IndexError:
            return N_pts

    def is_line_keepout(self, x1, y1, x2, y2):
        return self.N_line_keepout(x1, y1, x2, y2) > 0

    def is_line_forbidden(self, x1, y1, x2, y2):
        N_pts = int(np.ceil(((self.fac*x2-self.fac*x1)**2 + (self.fac*y2-self.fac*y1)**2)**.5))
        x, y = np.linspace(x1, x2, N_pts), np.linspace(y1, y2, N_pts)
        try:
            lineval = self.forbid_map[(y*self.fac+self.ref[0]).astype(int), (x*self.fac+self.ref[1]).astype(int)]

        except IndexError:
            # start/end outside map
            return True

        if np.sum(lineval) == 0:
            # not passing anything - all is good
            return False

        elif self.is_point_keepout(x[0], y[0]):
            return True

        elif lineval[-1] or not lineval[0]:
            # not starting from a forbidden point or going into one -> move is forbidden
            return True

        else:
            # going from forbidden to allowed - this is the tricky case. We need to check if path is passing though
            # a forbidden region
            print('Cannot easily determine line viability... walking it explicitly...')
            ko_val = self.keepout_map[(y*self.fac+self.ref[0]).astype(int), (x*self.fac+self.ref[1]).astype(int)]
            went_low = False
            previous_forbidden = True
            for xp, yp, fv, kv in zip(x, y, lineval, ko_val):
                if kv:
                    print(f'...found keep-out on the way at {xp}, {yp}.')
                    return True
                if not fv:
                    previous_forbidden = False
                if fv and (not went_low) and previous_forbidden:
                    went_low = True
                    print(f'Unforbidden at {xp}, {yp}')
                if fv and went_low and (not previous_forbidden):
                    print(f'Forbidden again at {xp}, {yp} -> path is forbidden!')
                    return True

        return False

    def validate(self, x1, y1, x2=None, y2=None, override_stop=False, override_state=False):
        # assumes forbidden for lines, and keepout for points
        if (x2 is not None) and (y2 is not None):
            if self.is_line_forbidden(x1, y1, x2, y2):
                if (self.stop and (not override_stop)) and (self.drive is not None):
                    self.drive.stop()
                if (self.state is not None) and (not override_state):
                    self.state.set_error()
                raise KeepoutError(current_pos=(x1, y1), target_pos=(x2, y2), mqtt_client=self.mqtt_client)
        else:
            if self.is_point_keepout(x1, y1):
                if (self.stop and (not override_stop)) and (self.drive is not None):
                    self.drive.stop()
                if (self.state is not None) and (not override_state):
                    self.state.set_error()
                raise KeepoutError(current_pos=(x1, y1), target_pos=None, mqtt_client=self.mqtt_client)

    def override(self, func):
        def move_without_limits(*args, **kwargs):
            stp = self.stop
            try:
                self.stop = False
                print(f'Force keepout: {self.stop}')
                func(*args, **kwargs)
            finally:
                self.stop = stp
                print(f'Force keepout: {self.stop}')

        return move_without_limits

    def set_override(self, value):
        if int(value):
            print('Deactivating Putzini interlock!')
            self.stop = False
        else:
            print('Activating Putzini interlock!')
            self.stop = True
