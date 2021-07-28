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
import math
import re

import json
import random

from sys import argv
import os
import simpleaudio as sa
import subprocess
# import skimage.io
# import skimage.draw
import imageio

import numpy as np

import time

from putzini_lamp import PutziniLamp
from putzini_config import PutziniConfig
from putzini_nav import PutziniNav2
from putzini_cam import PutziniCam
from putzini_state import PutziniState
from putzini_keepout import PutziniKeepoutArea, KeepoutError
from inspect import isawaitable
import csv
import logging

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
        await asyncio.sleep(0.8)
        self.set_aux(0)
        await asyncio.sleep(3)
        self.move(-30000)
        self.calibrated = True

    def move(self, steps , speed = 250):
        self.speed = speed
        frame = struct.pack(">lB",int(steps), self.speed)
        self.writer.write(frame)

    def set_position(self, position):
        #convert to steps 
        position = float(position)/0.005
        if self.calibrated:
            self.move(position-self.current_pos)
            self.current_pos = position
        else:
            print("Error: Neck not calibrated")
        
    def set_zero(self, _ ):
        self.current_pos = 0
        self.calibrated = True
        
    def set_vacuum(self, onoff):
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

class PutziniSound:
    def __init__(self, dev_name):
        self.wave = None
        self.play_obj = None
        self.dev = dev_name
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

        print(f'Loading wave file {fn}...')
        self.wave = sa.WaveObject.from_wave_file(fn)

        fn_lbl = fn.rsplit('.', 1)[0] + '.txt'
        if os.path.isfile(fn_lbl):
            print(f'Found corresponding label file {fn_lbl}')
            with open(fn_lbl, newline='') as fh:
                reader = csv.DictReader(delimiter='\t')

        assert subprocess.run(['pactl', 'set-sink-volume', self.dev, f'{int(vol)}%']).returncode == 0            
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
        self.config = PutziniConfig()
        self.state = PutziniState(mqtt_client)
        self.drive = PutziniDrive(mqtt_client)
        self.keepout = PutziniKeepoutArea(mqtt_client, self.config, self.drive, self.state)
        self.cam = PutziniCam(mqtt_client)
        self.nav = PutziniNav2(mqtt_client, self.state, self.config, self.keepout, self.cam)
        # self.nav2 = PutziniNav2(mqtt_client, self.state)
        self.lamp = PutziniLamp()
        self.neck = PutziniNeckAndVacuum()
        self.sound = PutziniSound(dev_name='alsa_output.usb-Generic_TX-Hifi_Type_C_Audio-00.analog-stereo')
        self.mqtt_client = mqtt_client
        
        self.putz_per_degree = 50
        self.putz_per_degree_array = np.ones(10)*self.putz_per_degree
        
        self.putz_per_meter = 17241*0.9

    async def start(self):
        d = asyncio.ensure_future(self.drive.connect())
        n = asyncio.ensure_future(self.nav.connect())
        l = asyncio.ensure_future(self.lamp.connect())
        m = asyncio.ensure_future(self.neck.connect())
        c = asyncio.ensure_future(self.cam.start())
          
        await asyncio.gather(d, n, l, m, c)
        # await asyncio.gather(d, n, m)
    
    async def turn_absolute(self, angle, speed=60, accuracy=4, slow_angle=30, speed_lim=25):
        angle = int(angle)

        # print(f'Turn to {angle} from {self.nav.get_angle()}, speed {speed}, acc. {accuracy}, slowdown below {slow_angle}')
        fudge = 1
        prev_a = 0
        close_to_target = False

        while True:
            # old_angle = self.nav.get_angle()
            # print('Reading previous angle')
            old_angle = (await asyncio.gather(self.nav.get_new_angle()))[0]
            
            # calculate the needed relative turn 
            # https://stackoverflow.com/questions/1878907
            a = angle - old_angle
            a = (a + 180) % 360 - 180

            # overshoot damper
            if a*prev_a < 0:
                fudge = max(0.1,fudge*0.7)
            prev_a = a

            # print(f"{old_angle:.2f} to {angle:.2f} => delta={a:.2f}; act spd=[{self.drive.meas_speed_r:.1f}, {self.drive.meas_speed_l:.1f}]; spd={speed}; fudge={fudge}")

            if abs(a) < slow_angle:
                speed = min(speed,50)
                            
            if (abs(a) < accuracy) and (abs(self.drive.meas_speed_r) < speed_lim) and (abs(self.drive.meas_speed_l) < speed_lim):
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

    async def move_absolute(self, x, y=0, speed=60, accuracy=10, evade=True):
        #TODO adaptive rotation accuracy
        x= int(x) / 100
        y= int(y) / 100
        accuracy = int(accuracy) / 100
        
        while True: 
            start = self.nav.get_position()
            end = np.array([x,y])

            # print(start, end)
            try:
                self.keepout.validate(start[0], start[1], end[0], end[1])
            except KeepoutError as err:
                if evade:
                    print(f'Direct move to {end*100} cm is forbidden. Attempting to go via waypoints.')
                    await self.move_circle(speed=speed, stride=1, exit_x=x*100, exit_y=y*100)
                else:
                    raise err

            diff = end-start
            distance = np.linalg.norm(diff)
            
            if distance < accuracy:
                break
            
            a = math.atan2(diff[1], diff[0])/math.pi*180
            if speed < 0:
                a += 180
            
            print (f"move from {start} to {end}: turn to {a}° and drive {distance}m.")
            await self.turn_absolute(a, np.abs(speed), accuracy=max(3,10*min(distance,1)))
            
            if distance > 1:
                self.nav.tell_straight_move_start()
            self.drive.move(min(distance,1)*self.putz_per_meter, -speed)
            
            await self.drive.finished
            self.nav.tell_straight_move_done()

    async def move_relative(self, x, y, speed=60, accuracy=10):
        x = int(x)/100
        y = int(y)/100
        cur_pos = self.nav.get_position()
        sa, ca = np.sin(self.nav.get_angle()*np.pi/180), np.cos(self.nav.get_angle()*np.pi/180)
        delta = np.matmul([[ca, -sa], [sa, ca]], [x, y])
        final_pos = cur_pos + delta
        print(f'Relative move from {cur_pos} by [{x}, {y}] w.r.t. Putzini.')
        print(f'Results in absolute move by {delta} to {final_pos}.')
        await self.move_absolute(final_pos[0]*100, final_pos[1]*100, speed=speed, accuracy=accuracy, evade=False)
            
    async def move_random(self, speed=60, xmin=0, xmax=0, ymin=0, ymax=0):
        while True:
            next_x = random.randint(xmin, xmax)
            next_y = random.randint(ymin, ymax)
            print(f'Random step to {next_x}, {next_y}, speed: {speed}')
            try:
                await self.move_absolute(next_x, next_y, speed, accuracy=40, evade=False)
            except KeepoutError as err:
                print(f'Random move is impossible: {err}. Trying another...')
                pass
            
    async def move_circle(self, speed=60, stride=1, exit_x=None, exit_y=None):
        speed = int(speed)
        N_steps = len(self.config.waypoint_x)
        waypoints = np.stack([self.config.waypoint_x, self.config.waypoint_y]).T/100
        distances = ((self.nav.get_position().reshape(1,2) - waypoints)**2).sum(axis=1)**.5
        cur_step = np.argmin(distances)
        print(f'Starting circle with {N_steps} waypoints. Distances are {distances}; closest point is {cur_step}')
        while True:
            print(f'Moving to step {cur_step} at {waypoints[cur_step,:]}')
            await self.move_absolute(100*waypoints[cur_step, 0], 100*waypoints[cur_step, 1], 
                speed=speed, accuracy=40, evade=False)
            cur_step = (cur_step + stride) % N_steps
            if (exit_x is not None) and (exit_y is not None):
                pos = self.nav.get_position()
                if not self.keepout.is_line_forbidden(pos[0], pos[1], exit_x/100., exit_y/100.):
                    print(f'Path to exit position {exit_x}, {exit_y} is open. Moving there and stopping.')
                    await self.move_absolute(exit_x, exit_y, speed=speed, accuracy=10, evade=False)
                    break
            
    async def move_straight(self, distance=0, speed=60, xmin=None, xmax=None, ymin=None, ymax=None):
        #TODO CATCH INVALID END COORDINATE
        distance, xmin, xmax, ymin, ymax = (int(p) / 100 for p in (distance, xmin, xmax, ymin, ymax))
        dist_putz = self.putz_per_meter*distance
        ini_pos = self.nav.get_position()
        final_pos = ini_pos + [np.cos(self.nav.get_angle()*np.pi/180)*distance, np.sin(self.nav.get_angle()*np.pi/180)*distance]
        speed = np.abs(int(speed))
        if (xmin is not None and (final_pos[0] < xmin)) or (xmax is not None and (final_pos[0] > xmax)) \
                or (ymin is not None and (final_pos[1] < ymin)) or (ymax is not None and (final_pos[1] > ymax)):
            raise ValueError(f'Final position {final_pos} of straight move would be outside bounds')
        print(f'Straight move by {distance} m = {dist_putz:.3f} putz. Estimated final position is {final_pos}.')
        self.keepout.validate(x1=ini_pos[0], y1=ini_pos[1], x2=final_pos[0], y2=final_pos[1])
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
            except KeepoutError as err:
                if random:
                    print(f'Back-forth move is impossible: {err}. Trying another...')
                    continue
                else:
                    print('Requested back-forth move is impossible.')
                    raise err
                    
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

            if "height" in cmd and cmd["height"] != None:
                h = int(cmd["height"])
                putzini.neck.set_position(h)
                print(f"setting neck to {h}")


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
                elif cmd["move"].startswith("moveCircle"):
                    pp = parse_command("moveCircle", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.state.set_active()
                    move_task = asyncio.ensure_future(putzini.move_circle(*pp))                                                                            
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

    async def move_without_limits(*args, **kwargs):
        stp = putzini.keepout.stop
        try:
            putzini.keepout.stop = False
            print(f'Force keepout: {putzini.keepout.stop}')
            putzini.drive.move(*args, **kwargs)
            await putzini.keepout.drive.finished
        finally:
            putzini.keepout.stop = stp
            print(f'Force keepout: {putzini.keepout.stop}')

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
        # tasks.add(asyncio.ensure_future(await_func_with_msg(messages, move_without_limits)))
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move)))
        
        manager = client.filtered_messages("putzini/turn")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/turn")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.turn)))
        
        manager = client.filtered_messages("putzini/override")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/override")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.keepout.set_override)))

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
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.lamp.set_head)))
        
        manager = client.filtered_messages("putzini/neck")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/neck")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.set_position)))

        manager = client.filtered_messages("putzini/neck_pos")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/neck")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.set_zero)))

        manager = client.filtered_messages("putzini/vacuum")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/vacuum")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.set_vacuum)))
        
        manager = client.filtered_messages("putzini/store_calib")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/store_calib")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.nav.store_calib)))
        
        manager = client.filtered_messages("putzini/reset_sensor")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/reset_sensor")
        tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.nav.write_calibration_to_sensor)))

        manager = client.filtered_messages("putzini/force_idle")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/force_idle")
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.state.set_idle)))

        manager = client.filtered_messages("putzini/commands")
        messages = await stack.enter_async_context(manager)
        await client.subscribe("putzini/commands")
        tasks.add(asyncio.ensure_future(parse_json_commands(messages, putzini)))


        await asyncio.gather(*tasks)
            

        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()
