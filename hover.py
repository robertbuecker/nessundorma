#!/usr/bin/env python3

import struct


#typedef struct{
#   uint16_t start;
#   int16_t 	cmd1;
#   int16_t 	cmd2;
#   int16_t 	speedR_meas;
#   int16_t 	speedL_meas;
#   int16_t 	batVoltage;
#   int16_t 	boardTemp;
#   uint16_t cmdLed;
#   uint16_t checksum;
#} SerialFeedback;


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

import numpy as np

from scipy.spatial.distance import euclidean
import time
from scipy.optimize import minimize
import board
import adafruit_bno055

class PutziniState:
    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client
        self.pos = np.zeros(3)
        self.alpha = 0
        self.action = 'Idle'
    
    def set_active(self):
        self.action = 'Moving'
        self.publish()
        
    def set_idle(self,bla=""):
        self.action = "Idle"
        self.publish()
        
    def set_position_with_alpha(self, pos, alpha):
        self.pos = pos
        self.alpha = alpha 
        self.publish()
        
    def publish(self):
        asyncio.ensure_future(self.mqtt_client.publish("putzini/state", json.dumps({'action': self.action, 'posX': self.pos[0], 'posY': self.pos[1], 'alpha': self.alpha[2]}), qos=0))  


class PutziniKeepoutArea:
    def __init__(self, keepout_img):
        # the image should have 1px per mm
        self.img = skimage.io.imread(keepout_img)
        self.ref = self.img.shape[0]/2, self.img.shape[1]/2
        
    def _transform_to_image_system(self, x, y):
        pass
        
    def is_line_keepout(self, x1, y1, x2, y2):
        r, c = skimage.draw.line(int(x1), int(y1), int(x2), int(y2))
        return np.sum(self.img[r, c] != np.zeros(4)) > 0
    
class PutziniLamp:
    def __init__(self):
        self.l = {"back":{"r":255,"g":255,"b":100},"front":{"r":0,"g":1,"b":0,"w":0}}
        self.h = 127
        pass
    
    async def connect(self, url = '/dev/serial/by-path/platform-70090000.xusb-usb-0:2.3:1.0-port0'):
        _ , self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=115200)
        asyncio.ensure_future(self._writer_task())
    
    async def _writer_task(self):
        while True:
            await asyncio.sleep(100e-3)
            frame = struct.pack("BBBB",self.h,self.l["back"]["r"],self.l["back"]["g"],self.l["back"]["b"])
            for x in range(0,12):
                frame += struct.pack("BBBB",self.l["front"]["r"],self.l["front"]["g"],self.l["front"]["b"],self.l["front"]["w"])
            
            self.writer.write(frame)
    
    def set_lamp(self,l):
        if "front" in l:
            self.l["front"] = l["front"]
        if "back" in l:
            self.l["back"] = l["back"]
    
    def set_head(self, h):
        print('Trying to set head to', h)
        self.h = int(h)


class PutziniNeckAndVacuum:
    def __init__(self):
        self.current_pos = 0
        self.calibrated = False
        self.speed = 127
        self.vacuum = 0
        self.aux = 0
        pass
  
    async def connect(self, url="/dev/serial/by-path/platform-70090000.xusb-usb-0:2.1:1.0-port0", baudrate=115200):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)

    def move(self, steps , speed = 127):
        self.speed = speed
        frame = struct.pack(">lB",int(steps), self.speed)
        self.writer.write(frame)
        
    def set_position(self, position):
        #convert to steps 
        position = position/0.005
        if self.calibrated:
            self.move(position-self.current_pos)
            self.current_pos = position
        else:
            print("Error: Neck not calibrated")
        
    def set_zero(self):
        self.current_pos = 0
        self.calibrated = True
        
    def set_vacuum(self,onoff):
        """0 == off, 1 == on"""
        
        self.vacuum = int(onoff)
        frame = struct.pack(">lBB",0, self.speed, self.vacuum)
        self.writer.write(frame)
        
    def set_aux(self, onoff):
        self.aux = int(onoff)
        frame = struct.pack(">lBBB",0, self.speed, self.vacuum, self.aux)
        self.writer.write(frame)

class PutziniDrive:

    START_FRAME = 0xABCD
    
    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client
    
        self.meas_speed_l = 0
        self.meas_speed_r = 0
        self.bat_voltage = 0
        self.temperature = 0
        
        self.speed_l = 0
        self.speed_r = 0
        
        self.distance_to_move_l = 0
        self.distance_to_move_r = 0
        
        self.read_error = True
        
        self.loop = asyncio.get_event_loop()
        self.finished = self.loop.create_future()
        self.finished.set_result(None)
        self.period = 50e-3
        self.wheel_balance = 1.05 # >1 -> make it move rather to the right

    async def connect(self, url='/dev/serial/by-path/platform-70090000.xusb-usb-0:2.4:1.0-port0', baudrate=38400):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)       
        asyncio.ensure_future(self._reader_task())
        asyncio.ensure_future(self._writer_task())
         
    async def _reader_task(self):
        msg=b''
        i = 0
        while True:
            msg = await self.reader.read(18)
            if len(msg) == 18:
                msg=struct.unpack('HhhhhhhHH',msg)
                if msg[0] == PutziniDrive.START_FRAME and msg[8] == (msg[0] ^ msg[1] ^ msg[2] ^ msg[3] ^ msg[4] ^ msg[5] ^ msg[6] ^ msg[7]) & 0xffff:
                    self.read_error = False
                    self.meas_speed_l = msg[4]*2
                    self.meas_speed_r = msg[3]*-2
                    self.bat_voltage = msg[5]/100
                    self.temperature = msg[6]/10
                    
                    self.distance_to_move_l -= abs(self.meas_speed_l)
                    self.distance_to_move_r -= abs(self.meas_speed_r)
                    
                    i += 1
                    if i > 200:
                        asyncio.ensure_future(self.mqtt_client.publish("putzini/v_batt", self.bat_voltage, qos=0))
                        asyncio.ensure_future(self.mqtt_client.publish("putzini/temp", self.temperature, qos=0))
                        i=0
                    
                    #print(f"cmd1 {msg[1]}, cmd2: {msg[2]}, spdR:{msg[3]}, spdL:{msg[4]}, batVoltage: {msg[5]/100}, temp: {msg[6]/10}")
                else:
                    self.read_error = True
	                
    async def _writer_task(self):
        while True:
            await asyncio.sleep(self.period)
            speed_r = 0
            speed_l = 0
            
            if not self.read_error and self.distance_to_move_l > 0:
                speed_l = self.speed_l
                
            if not self.read_error and self.distance_to_move_r > 0:
                speed_r = self.speed_r
            
            if not self.read_error and self.distance_to_move_r <= 0 and self.distance_to_move_l <= 0:
                if not self.finished.done():
                    self.finished.set_result(None) 
            
            # set
            self.moving = speed_r > 0 or speed_l > 0
            
            # WHEEL FUDGE FACTOR
            # for motor imbalance. Note that "r" means the LEFT wheel seen from behind!
            speed_l = int(speed_l * self.wheel_balance)
            speed_r = int(speed_r * 1/self.wheel_balance)
        
            frame = struct.pack('HhhH',PutziniDrive.START_FRAME, speed_r, speed_l, (PutziniDrive.START_FRAME ^ speed_r ^ speed_l) & 0xffff)
            self.writer.write(frame)

    def stop(self):
        print('IMMEDIATE STOP')
        self.distance_to_move_r = 0
        self.distance_to_move_l = 0


    def move_r(self, dist):
        self.distance_to_move_r = abs(int(dist))
        if self.finished.done():
            self.finished = loop.create_future()
    
    def move_l(self,dist):
        self.distance_to_move_l = abs(int(dist))
        if self.finished.done():
            self.finished = loop.create_future()
        
    def set_speed_l(self, speed):
        self.speed_l = int(speed)
        
    def set_speed_r(self, speed):
        self.speed_r = int(speed)
        
    def turn(self, angle, speed=60):
        if int(angle) < 0:
            speed*=-1
        self.set_speed_l(-int(speed))
        self.set_speed_r(int(speed))
        self.move_r(int(angle))
        self.move_l(int(angle))
        if self.finished.done():
            self.finished = loop.create_future()
            
    def move(self, dist, speed=60):
        if int(dist) < 0:
            speed *=-1
        self.set_speed_r(speed)
        self.set_speed_l(speed)
        self.distance_to_move_l = abs(int(dist))
        self.distance_to_move_r = abs(int(dist))
        if self.finished.done():
            self.finished = loop.create_future()


class PutziniNav2:
    # Anchor-based system

    def __init__(self, mqtt_client, putzini_state):
        # print('Position class started')
        # asyncio.ensure_future(self.connect())
        self.initialized = False
        self.mqtt_client = mqtt_client
        self.state = putzini_state

        if os.path.exists('putziniNav.yml'):
            with open('putziniNav.yml') as fh:
                self.opts = yaml.load(fh)
            self.opts = {} if self.opts is None else self.opts
        else:
            self.opts = {}

        self.ids = {'anchor_1': 'B4DE',
                    'anchor_2': 'B4D3',
                    'anchor_3': 'B4D9',
                    'tag': 'B521'}
        
        self.anchor_idx = {b'B4DE': 0, b'B4D3': 1, b'B4D9': 2}
        
        self.anchor_pos = np.array([[-260, -400, 0],
                           [260,-400, 0],
                           [0,-35, 0]],dtype=float)/100
 
        self.distances = np.array([0.,0.,0.])
        self.distances_sig = np.array([0.,0.,0.])
        self._distance_buffer = []
        self.position = self.anchor_pos.mean(axis=0)
        self.position[2] = 0
        self.RT_rp = np.eye(4)
        self.sensor = None
        self.alpha = 0
        self.t_last_angle = 0
        self.timestamp = -1.
        self.avg_len = 20
        self.t_update = 50

    async def connect(self, url='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', baudrate=512000):
        i2c = board.I2C()
        i2c.init(board.SCL_1,board.SDA_1, 800)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        print('I2C orientation sensor connected.')
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)  
        print('Master anchor connected.')     
        asyncio.ensure_future(self._reader_task())
        await self.start_ranging()
            
    async def start_ranging(self):
        await self.stop_ranging()
        self.writer.write(b'$PL,\r\n')
        # config_string = f'$PK,{self.ids["anchor_1"]},2,1,{self.ids["anchor_2"]},{self.ids["anchor_3"]},{self.ids["tag"]},\r\n'
        config_string = f'$PK,{self.ids["tag"]},0,3,{self.ids["anchor_1"]},{self.ids["anchor_2"]},{self.ids["anchor_3"]},\r\n'
        config_string = config_string.encode('utf-8')
        # print(config_string)
        self.writer.write(config_string)
        self.writer.write(b'$PS,\r\n')
        print('Ranging configured and started.')
        asyncio.ensure_future(self.update_position())
        
    async def stop_ranging(self):
        self.writer.write(b'$PG,')
            
    async def update_position(self):
        
        while True:

            ela = time.time() - self.timestamp
            # print(round(ela*1000))
            await asyncio.sleep(self.t_update/1000. - ela)

            self.alpha = self.sensor.euler
            self.alpha = (-self.alpha[0], self.alpha[1], self.alpha[2])            
            self.timestamp = time.time()

            if len(self._distance_buffer) == 0:
                # print('Distance buffer is empty. WTF?')
                continue

            _distances = np.stack(self._distance_buffer)
            self._distance_buffer = []
            N_valid = (1-np.isnan(_distances)).sum(axis=0)
            avg = np.nanmean(_distances, axis=0)
            self.distances[N_valid >= 2] = avg[N_valid >= 2]   

            include_z = True
            t0 = time.time()

            if include_z:
                def error(x):
                    dist_err = ((self.anchor_pos - x.reshape(1,3))**2).sum(axis=1)**.5 - self.distances
                    # print((dist_err**2/dist**2))
                    f = (dist_err**2/self.distances).sum()
                    return f
                self.position = minimize(error, self.position, method='BFGS').x

            else:
                def error(x):
                    dist_err = ((self.anchor_pos - np.concatenate([x.reshape(1,2), np.zeros((1,1))], axis=1))**2).sum(axis=1)**.5 - self.distances
                    # print((dist_err**2/dist**2))
                    f = (dist_err**2/self.distances).sum()
                    return f
                self.position[:2] = minimize(error, self.position[:2], method='BFGS').x        

            # self.position = pos_solve(self.distances, self.anchor_pos, self.position)/100.
            print(f'N = {N_valid}; d = {(self.distances*100).round(1)} cm; x = {(self.position*100).round(1)} cm; tOpt = {(time.time()-t0)*1000:.0f} ms')
            # dirty fix: just inverting in-plane angle for now
            c, s = np.cos(self.alpha[0]/180*np.pi), np.sin(self.alpha[0]/180*np.pi)
            self.RT_rp = np.array([[c,-s,0,self.position[0]], 
                                    [s,c,0,self.position[1]], 
                                    [0,0,1,self.position[2]], 
                                    [0,0,1,0]])
            asyncio.ensure_future(self.mqtt_client.publish("putzini/distances", f'N = {N_valid}; d = {self.distances.round(4)}', qos=0))
            asyncio.ensure_future(self.mqtt_client.publish("putzini/position", repr(self.RT_rp), qos=0))
            self.state.set_position_with_alpha(self.position, self.alpha)
            sensordat = {
                        'B': self.sensor.magnetic,
                        'aA': tuple(x/np.pi*360 for x in self.sensor.gyro),
                        'aL': self.sensor.linear_acceleration,
                        'g': self.sensor.gravity,
                        'a': self.sensor.acceleration}
            # print(sensordat)
            asyncio.ensure_future(self.mqtt_client.publish("putzini/sensordata", json.dumps(sensordat)))

    async def _reader_task(self):
        msg=b''
        ii = 0
        print('Positioning reader task started')
        while True:
            msg = await self.reader.readline()
            # msg = msg.strip().decode()
            # print(msg)
            try:
                cmd, par = msg.strip().split(b',',1)
            except:
                print(f'Failing to split message: {msg}')
                continue
            # cmd, par = cmd.decode(), par.decode()
            # print('Received:',cmd, par)
            if cmd == b'$PX':
                print(f'Ping received: {par}')
                
            elif cmd == b'$PD':
                ii += 1
                # await asyncio.sleep(0.1)
                new_dist = np.nan*np.ones(3)
                try:
                    tag_id, a1_dist, a2_dist, a3_dist, udata, _ = par.split(b',',5)
                    d1, d2, d3 = int(a1_dist, 16), int(a2_dist, 16), int(a3_dist, 16)
                    if not d1 == 0:
                        new_dist[self.anchor_idx[tag_id]] = float(d1)/100.
                    self._distance_buffer.append(new_dist)
                except:
                    print(f'Could not decode distance message: {par}')
                    pass
                                            
            elif cmd == b'$PS':
                print(f'Ranging started.')
                
            elif cmd == b'$PG':
                print(f'Ranging stopped.')
                
            elif cmd == b'$PW':
                print(f'Configuration received: {par}')
    
    def get_position(self):
        return self.position[:2]

    def get_angle(self):
        return self.alpha[0]

    async def get_new_angle(self):
        while self.timestamp == self.t_last_angle:
            await asyncio.sleep(0.04)
        self.t_last_angle = self.timestamp
        return self.alpha[0]
    
    def get_position(self):
        return self.position[:2]

class PutziniNav:
    def __init__(self, mqtt_client, putzini_state):
        self.state = putzini_state
        self.mqtt_client = mqtt_client
        self.detected = 0
        if os.path.exists('putziniNav.yml'):
            with open('putziniNav.yml') as fh:
                self.opts = yaml.load(fh)
            self.opts = {} if self.opts is None else self.opts
        else:
            self.opts = {}

        self.init_reference_system()

        # Transform convention:
        # _ba = "a as seen from b", or "transforms a coordinates to b coordinates"

        # MEASUREMENT
        self.RT_cm = np.eye(4) # marker as seen from camera (as received from ArUco)
        self.RT_pr = np.eye(4) # Reference seen from Putzini (stable intermediate)
        self.RT_rp = np.eye(4) # Putzini as seen from reference/room (as broadcasted via MQTT) 
        self.RT_pm = np.eye(4)
        self.timestamp = -1.
        self.t_last_angle = -1.

    def init_reference_system(self):
        # reference as seen from ArUco system. The flip of the z axis (180 deg around y)
        # is always done, even if no additional calibration is present

        if 'marker-system' in self.opts:
            x, y, z, angle = (float(self.opts['marker-system'][k]) for k in ['x', 'y', 'z', 'angle'])
            ca, sa = np.cos(angle*np.pi/180), np.sin(angle*np.pi/180)
            reference = np.array([[ca, -sa, 0, x],
                                [sa, ca, 0, y],
                                [0, 0, 1, z],
                                [0, 0, 0, 1]])
            print('Setting reference system w.r.t. marker system to:')
            print(reference)
            self.RT_mr = np.matmul(reference, np.diag([1, -1, -1, 1]))

        else:
            self.RT_mr = np.diag([1, -1, -1, 1])

        # cam on Putzini is hard-coded: rotated by about 90 deg and 15 cm above wheel hubs
        self.RT_pc = np.array([[0,1,0,0], [-1,0,0,0], 
                        [0,0,1,0.15], [0,0,0,1]])             

    async def start(self):
        cmd = 'aruco_dcf_mm' if len(argv) > 1 and argv[1] == 'gui' else 'aruco_dcf_mm_nogui'
        # self.proc = await asyncio.create_subprocess_exec(cmd,'live:0','calib_usbgs/map_rfa.yml','calib_usbgs/usbgs.yml','-f ','arucoConfig.yml','-r','16', stdout=asyncio.subprocess.PIPE)
        # asyncio.ensure_future(self._reader_task())

    async def _reader_task(self):
        while True:
            ln = (await self.proc.stdout.readline()).decode()
            if ln.startswith('|@'):
                status = ln
            elif ln.startswith('ArUco Detected'):
                self.detected = int(ln.split()[2])
            elif ln.startswith('['):
                ln1 = (await self.proc.stdout.readline()).decode()
                ln2 = (await self.proc.stdout.readline()).decode()
                ln3 = (await self.proc.stdout.readline()).decode()
            
                RT_str = f'[{ln.replace(";", "],")} ' + \
                f'[{ln1.replace(";", "],")}' + \
                f'[{ln2.replace(";", "],")}' + \
                f'[{ln3}]'

                self.RT_cm = np.array(eval(RT_str))

                # transformation steps ensue...
                self.RT_pm = np.matmul(self.RT_pc, self.RT_cm)
                self.RT_pr = np.matmul(self.RT_pm, self.RT_mr)
                self.RT_rp = np.linalg.inv(self.RT_pr)

                xform = transform.Rotation.from_dcm(self.RT_rp[:3,:3])
                alpha = xform.as_euler('XYZ')*180/np.pi

                if 'out-of-plane-limit' in self.opts and max(alpha[:2]) > float(self.opts['out-of-plane-limit']):
                    wstr = f'WARNING: out of plane angles {alpha[:2]} exceed limit.'
                    # asyncio.ensure_future(self.mqtt_client.publish("putzini/state",json.dumps({'navstatus': wstr}),qos=0))  
                    print(wstr)
                else:
                    self.position = self.RT_rp[:3,-1]
                    self.alpha = alpha
                    self.timestamp = time.time()
                    # asyncio.ensure_future(self.mqtt_client.publish("putzini/state",json.dumps({'navstatus': 'OK'}),qos=0))  

                asyncio.ensure_future(self.mqtt_client.publish("putzini/position",repr(self.RT_rp),qos=0))
                self.state.set_position_with_alpha(self.position, self.alpha)
                
    # def zero_here(self): # sets position and Z-angle of c and m coordinate systems equal

    def get_angle(self):
        return self.alpha[2]

    async def get_new_angle(self):
        while self.timestamp == self.t_last_angle:
            await asyncio.sleep(0.04)
        self.t_last_angle = self.timestamp
        return self.alpha[2]
    
    def get_position(self):
        return self.position[:2]

    def set_reference_position(self, clear=False, x=0, y=0):    
        if clear:
            self.opts['marker-system'] = {'x': 0, 'y': 0, 
                'z': 0, 'angle': 0}
        else:
            RT_pm_moved = self.RT_pm
            RT_pm_moved[:3,-1] = RT_pm_moved[:3,-1] + np.array([x/100,y/100,0])
        
            RT_calib = np.matmul(np.linalg.inv(RT_pm_moved), np.diag([1, -1, -1, 1]))
            angles = transform.Rotation.from_dcm(RT_calib[:3,:3]).as_euler('XYZ')*180/np.pi

            self.opts['marker-system'] = {'x': float(RT_calib[0,-1]), 'y': float(RT_calib[1,-1]), 
                'z': float(RT_calib[2,-1]), 'angle': float(angles[2])}

        self.init_reference_system()

        with open('putziniNav.yml', 'w') as fh:
            yaml.dump(self.opts, fh)

class PutziniSound:
    def __init__(self, dev_name):
        self.wave = None
        self.play_obj = None
        try:
            # Activate the proper sound device
            print('Activating sound device: ', dev_name)
            assert subprocess.run(['pactl', 'set-default-sink', 
            dev_name]).returncode == 0
        except:
            print('Could not initialize sound device, sorry.')

    async def play(self, fn, loop=False, vol=None):
        if self.play_obj is not None and self.play_obj.is_playing():
            self.play_obj.stop()
            self.play_obj = None
            self.volume = vol

        print(f'Loading wave file {fn}...')
        self.wave = sa.WaveObject.from_wave_file(fn)
        self.play_obj = self.wave.play()
        self.fn = ''

        while True:
            if (self.play_obj is not None) and self.play_obj.is_playing():
                await asyncio.sleep(0.5)
            elif (self.play_obj is not None) and loop: 
                print(f'Restarting wave file {fn}.')
                self.play_obj = self.wave.play()
            else:
                print(f'Stopped wave file {fn}.')
                break

    def stop(self):
        if self.play_obj is not None:
            self.play_obj.stop()
            self.play_obj = None

        self.wave = None

class Putzini:
    def __init__(self, mqtt_client):
        self.state = PutziniState(mqtt_client)
        self.drive = PutziniDrive(mqtt_client)
        self.nav = PutziniNav2(mqtt_client, self.state)
        # self.nav2 = PutziniNav2(mqtt_client, self.state)
        self.lamp = PutziniLamp()
        self.lamp = None
        self.neck = PutziniNeckAndVacuum()
        self.sound = PutziniSound(dev_name='alsa_output.usb-Generic_TX-Hifi_Type_C_Audio-00.analog-stereo')
        self.mqtt_client = mqtt_client
        
        self.putz_per_degree = 50
        self.putz_per_degree_array = np.ones(10)*self.putz_per_degree
        
        self.putz_per_meter = 17241*0.9

    async def start(self):
        d = asyncio.ensure_future(self.drive.connect())
        n = asyncio.ensure_future(self.nav.connect())
        # l = asyncio.ensure_future(self.lamp.connect())
        l = None
        m = asyncio.ensure_future(self.neck.connect())
        
        # await asyncio.gather(d, n, l, m)
        await asyncio.gather(d, n, m)
    
    async def turn_absolute(self, angle, speed=60, accuracy=4, slow_angle=30):
        angle = int(angle)

        print(f'Turn to {angle} from {self.nav.get_angle()}, speed {speed}, acc. {accuracy}, slowdown below {slow_angle}')
        fudge = 1
        prev_a = 0
        close_to_target = False

        while True:
            # old_angle = self.nav.get_angle()
            print('Reading previous angle')
            old_angle = (await asyncio.gather(self.nav.get_new_angle()))[0]
            
            # calculate the needed relative turn 
            # https://stackoverflow.com/questions/1878907
            a = angle - old_angle
            a = (a + 180) % 360 - 180

            # overshoot damper
            if a*prev_a < 0:
                fudge = max(0.05,fudge*0.7)
            prev_a = a

            print(f"{old_angle:.2f} to {angle:.2f} => delta={a:.2f}; act spd=[{self.drive.meas_speed_r:.1f}, {self.drive.meas_speed_l:.1f}]; spd={speed}; fudge={fudge}")

            if abs(a) < slow_angle:
                speed = min(speed,50)
                            
            if (abs(a) < accuracy) and (abs(self.drive.meas_speed_r) < 5) and (abs(self.drive.meas_speed_l) < 5):
                if close_to_target == True:
                    self.drive.stop() # not working? no idea why!
                    break
                else:
                    # wait an extra round near the end...
                    await asyncio.sleep(0.2)
                    close_to_target = True
                    continue
            else:
                close_to_target = False

            # turn
            distance = a*self.putz_per_degree*fudge
            self.drive.turn(distance, speed)
            await asyncio.sleep(20e-3)       

        
        

    async def turn_relative(self, delta_angle, speed=60, accuracy=4, slow_angle=20):

        # delta_angle = int(delta_angle)
        await self.turn_absolute(delta_angle + self.nav.get_angle(), speed=speed, accuracy=accuracy, slow_angle=slow_angle)

    async def look_at(self, x, y, speed=60, accuracy=4):
        x= int(x) / 100
        y= int(y) / 100
        start = self.nav.get_position()
        end = np.array([x,y])

        diff = end-start
        a = math.atan2(diff[1], diff[0])/math.pi*180
        if speed < 0:
            a += 180
        
        print (f"Look from {start} at {end}: turn to {a}°")
        await self.turn_absolute(a, np.abs(speed), accuracy=accuracy)


    async def move_absolute(self, x, y=0, speed=60, accuracy=10):
        #TODO adaptive rotation accuracy
        x= int(x) / 100
        y= int(y) / 100
        accuracy = int(accuracy) / 100
        
        while True: 
            start = self.nav.get_position()
            end = np.array([x,y])

            diff = end-start
            distance = np.linalg.norm(diff)
            
            if distance < accuracy:
                break
            
            a = math.atan2(diff[1], diff[0])/math.pi*180
            if speed < 0:
                a += 180
            
            print (f"move from {start} to {end}: turn to {a}° and drive {distance}m.")
            await self.turn_absolute(a, np.abs(speed), accuracy=max(3,10*min(distance,1)))
            
            self.drive.move(min(distance,1)*self.putz_per_meter, -speed)
            await self.drive.finished


    async def move_relative(self, x, y, speed=60, accuracy=10):
        x = int(x)/100
        y = int(y)/100
        cur_pos = self.nav.get_position()
        sa, ca = np.sin(self.nav.get_angle()*np.pi/180), np.cos(self.nav.get_angle()*np.pi/180)
        delta = np.matmul([[ca, -sa], [sa, ca]], [x, y])
        final_pos = cur_pos + delta
        print(f'Relative move from {cur_pos} by [{x}, {y}] w.r.t. Putzini.')
        print(f'Results in absolute move by {delta} to {final_pos}.')
        await self.move_absolute(final_pos[0]*100, final_pos[1]*100, speed=speed, accuracy=accuracy)
            
    async def move_random(self, speed=60, xmin=0, xmax=0, ymin=0, ymax=0):
        while True:
            next_x = random.randint(xmin, xmax)
            next_y = random.randint(ymin, ymax)
            print(f'Random step to {next_x}, {next_y}, speed: {speed}')
            await self.move_absolute(next_x, next_y, speed)
            
    async def move_straight(self, distance=0, speed=60, xmin=None, xmax=None, ymin=None, ymax=None):
        distance, xmin, xmax, ymin, ymax = (int(p) / 100 for p in (distance, xmin, xmax, ymin, ymax))
        dist_putz = self.putz_per_meter*distance
        final_pos = self.nav.get_position() + [np.cos(self.nav.get_angle()*np.pi/180)*distance, np.sin(self.nav.get_angle()*np.pi/180)*distance]
        speed = np.abs(int(speed))
        if (xmin is not None and (final_pos[0] < xmin)) or (xmax is not None and (final_pos[0] > xmax)) \
                or (ymin is not None and (final_pos[1] < ymin)) or (ymax is not None and (final_pos[1] > ymax)):
            raise ValueError(f'Final position {final_pos} of straight move would be outside bounds')
        print(f'Straight move by {distance} m = {dist_putz:.3f} putz. Estimated final position is {final_pos}.')
        self.drive.move(dist_putz, speed=-speed)
        await self.drive.finished
        print(f'Straight move finished. Actual pos is {self.nav.get_position()} -> {self.nav.get_position() - final_pos} off ({((self.nav.get_position() - final_pos)**2).sum()**.5:.3f} m).')

    async def move_back_forth(self, range=0, max_angle=15, speed=60, random=1, xmin=None, xmax=None, ymin=None, ymax=None):
        curr_pos_linear = 0
        new_d = 0
        start_pos = self.nav.get_position()
        start_angle = self.nav.get_angle()

        while True:
            a = ((self.nav.get_angle() - start_angle) + 180) % 360 - 180
            print(f'Back-and-forth angle deviation is now {a}')
            if abs(a) > max_angle:
                await self.turn_absolute(start_angle, speed=50, accuracy=max_angle/2, slow_angle=max_angle)
            if new_d >= 0:
                new_d = np.random.randint(-range//2-curr_pos_linear, 0) if random else -range//2
            else:
                new_d = np.random.randint(1, range//2+1-curr_pos_linear) if random else range//2
            try:
                await self.move_straight(distance=new_d, speed=speed, xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)
            except ValueError as err:
                self.drive.stop()
                print('Error during straight section in back-forth:\n', err)
                print('Abandoning.')
                break
                # print('Resetting Putzini to initial position.')
                # await self.move_absolute(100*start_pos[0], 100*start_pos[1], speed=speed)
                # print('Moved back.')
            await self.drive.finished
            curr_pos_linear += new_d

    async def turn_forever(self, angle=1000000, speed=60):
        # TODO no new coordinates are sent
        self.drive.turn(angle*self.putz_per_degree, speed=speed)
        await self.drive.finished

async def call_func_with_msg(messages, func):
    async for message in messages:
        var = message.payload.decode("utf-8")
        func(var)
    
async def await_func_with_msg(messages, func):
    async for message in messages:
        var = message.payload.decode("utf-8")
        await func(var)


def parse_command(fun, npar, com):
    # parses command com of form fun(par, par, par,.....) with npar parameters, all integer.
    # Returns a tuple.
    parsed = re.search(fun+'\(' + '\s*,\s*'.join(npar*['([\d +-]+)']) + '\)', com)
    return None if parsed is None else tuple(int(p) for p in parsed.groups())  


async def parse_json_commands(messages, putzini: Putzini):
    move_task = asyncio.Future()
    async for message in messages:

        try:
            cmd = json.loads(message.payload.decode("utf-8"))                    
        except Exception as e:
            print(e)
            print(f"Error parsing {message.payload.decode('utf-8')}")
            cmd = {}

        try:            
            print (f"Executing: {cmd}")
            if "lamp" in cmd and cmd["lamp"] != None:
                l = cmd["lamp"]
                putzini.lamp.set_lamp(l)
                print(f"setting lampe to: {l}")

            if "head" in cmd and cmd["head"] != None:
                try:
                    h = int(cmd["head"])                   
                except ValueError as err:
                    if isinstance(cmd['head'], str):
                        print('Received Head Command:', cmd['head'])
                    else:
                        raise err
                else:
                    putzini.lamp.set_head(h)
                    print(f"setting head to: {h}") 

            if "vacuum" in cmd and cmd["vacuum"] != None:
                v = int(cmd["vacuum"])
                putzini.neck.set_vacuum(v)
                print(f"switching vacuum cleaner {'on' if v==1 else 'off'}")

            if 'audio' in cmd and cmd["audio"] != None:
                acmd = cmd["audio"]
                if isinstance(acmd, str) and (acmd.lower() == 'stop'):
                    putzini.sound.stop()
                else:
                    fn = os.path.join('/home/putzini/audio', acmd['folder'], acmd['file'])
                    print('Trying to play audio file', fn, 'at volume', acmd['vol'], 'as', ('loop' if acmd['loop'] else 'one-shot'))
                    asyncio.ensure_future(putzini.sound.play(fn, bool(acmd['loop'] ), acmd['vol']))

            if "move" in cmd and cmd["move"] != None:
                if cmd["move"] == "stop()":
                    move_task.cancel()
                    putzini.drive.stop()
                elif cmd["move"].startswith("moveToPos"):
                    pp = parse_command("moveToPos", 4, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_absolute(*pp))
                elif cmd["move"].startswith("moveByPos"):
                    pp = parse_command("moveByPos", 4, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    # await putzini.move_relative(*pp)
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_relative(*pp))      
                    print(pp)
                elif cmd["move"].startswith("moveToAngle"):
                    pp = parse_command("moveToAngle", 2, cmd["move"])
                    print(pp)
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.turn_absolute(pp[0],pp[1]))
                elif cmd["move"].startswith("moveByAngle"):
                    pp = parse_command("moveByAngle", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.turn_relative(pp[0],pp[1]))                
                elif cmd["move"].startswith("lookAtPos"):
                    pp = parse_command("lookAtPos", 3, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.look_at(pp[0],pp[1],pp[2]))                            
                elif cmd["move"].startswith("moveRandom"):
                    pp = parse_command("moveRandom", 5, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_random(*pp))
                elif cmd["move"].startswith("moveStraight"):
                    pp = parse_command("moveStraight", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_straight(*pp))     
                elif cmd["move"].startswith("moveBackForth"):
                    pp = parse_command("moveBackForth", 8, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_back_forth(*pp))                        
                elif cmd["move"].startswith("turnForever"):
                    pp = parse_command("turnForever", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.turn_forever(*pp))                                                        
                elif cmd["move"].startswith("setReferencePos"):
                    pp = parse_command("setReferencePos", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.nav.set_reference_position(False, pp[0], pp[1])         
                elif cmd["move"].startswith("clearReferencePos"):
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.nav.set_reference_position(clear=True)
                
                move_task.add_done_callback(putzini.state.set_idle)       

        except Exception as e:
            print(e)
            print(f"Error executing {message.payload.decode('utf-8')}:")


async def main():

    client = mqtt.Client("172.31.1.150")
    putzini = Putzini(client)

    
    async with AsyncExitStack() as stack:
        
        tasks = set()

        await stack.enter_async_context(client)
        await putzini.start()
        

        manager = client.filtered_messages("putzini/move_r")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/move_r")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move_r)))
        
        manager = client.filtered_messages("putzini/move_l")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/move_l")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move_l)))
        
        manager = client.filtered_messages("putzini/move")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/move")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move)))
        
        manager = client.filtered_messages("putzini/turn")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/turn")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.turn)))
        
        manager = client.filtered_messages("putzini/set_speed_r")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/set_speed_r")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.set_speed_r)))       
        
        manager = client.filtered_messages("putzini/set_speed_l")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/set_speed_l")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.set_speed_l)))
        
        manager = client.filtered_messages("putzini/turn_absolute")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/turn_absolute")
        tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.turn_absolute)))
        
        manager = client.filtered_messages("putzini/move_absolute")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/move_absolute")
        tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.move_absolute)))
        
        manager = client.filtered_messages("putzini/head")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/head")
        # tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.lamp.set_head)))
        
        manager = client.filtered_messages("putzini/neck")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/neck")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.move)))
                 
        manager = client.filtered_messages("putzini/vacuum")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/vacuum")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.set_vacuum)))

        manager = client.filtered_messages("putzini/commands")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/commands")
        tasks.add(asyncio.ensure_future(parse_json_commands(messages, putzini)))


        await asyncio.gather(*tasks)
            

        
if __name__ == '__main__':
    loop = asyncio.get_event_loop()      
    loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()
