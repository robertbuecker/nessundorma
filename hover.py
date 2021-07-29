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
from putzini_keepout import PutziniKeepoutArea, PathForbiddenError
from putzini_sound import PutziniSound
from putzini_drive import PutziniDrive
from inspect import isawaitable
import csv
import logging

logger = logging.getLogger('hover')
np.set_printoptions(precision=3, sign='+')

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
        frame = struct.pack(">lBB",int(steps), self.speed, self.vacuum)
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

class Putzini:
    def __init__(self, mqtt_client):
        self.logger = logger
        self.config = PutziniConfig()
        self.state = PutziniState(mqtt_client)
        self.drive = PutziniDrive(mqtt_client)
        self.keepout = PutziniKeepoutArea(mqtt_client, self.config, self.drive)
        self.cam = PutziniCam(mqtt_client)
        self.nav = PutziniNav2(mqtt_client, self.state, self.config, self.keepout, self.cam)
        self.lamp = PutziniLamp()
        self.neck = PutziniNeckAndVacuum()
        self.sound = PutziniSound(mqtt_client, dev_name='alsa_output.usb-Generic_TX-Hifi_Type_C_Audio-00.analog-stereo')
        self.mqtt_client = mqtt_client
        
        self.putz_per_degree = 50
        self.putz_per_degree_array = np.ones(10)*self.putz_per_degree
        
        self.putz_per_meter = 17241*0.9

        self.command_state = {}

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

        self.logger.info(f'Turn to {angle} from {self.nav.get_angle()}, speed {speed}, acc. {accuracy}, slowdown below {slow_angle}')
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

            self.logger.debug(f"{old_angle:.2f} to {angle:.2f} => delta={a:.2f}; act spd=[{self.drive.meas_speed_r:.1f}, {self.drive.meas_speed_l:.1f}]; spd={speed}; fudge={fudge}")

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
        
        self.logger.info(f"Look from {start.round(3)*100} at {end.round(3)*100}: turn to {a}°")
        await self.turn_absolute(a, np.abs(speed), accuracy=accuracy)

    async def move_absolute(self, x, y=0, speed=60, accuracy=10, evade=True):
        #TODO adaptive rotation accuracy
        x= int(x) / 100
        y= int(y) / 100
        accuracy = int(accuracy) / 100
        target = np.array([x,y])
        ii = 0
        while True: 
            start = self.nav.get_position()
            end = np.array([x,y])

            # print(start, end)
            try:
                self.keepout.validate(start[0], start[1], end[0], end[1])
            except PathForbiddenError as err:
                if evade and not self.keepout.is_point_forbidden(end[0], end[1]):
                    self.logger.warning(f'move_abs: Direct move from {start*100} to {end*100} cm is forbidden. Attempting to go via waypoints.')
                    #TODO better logic about circle direction
                    await self.move_circle(speed=speed, stride=1, exit_x=x*100, exit_y=y*100)
                else:
                    raise err

            diff = end-start
            distance = np.linalg.norm(diff)
            
            if distance < accuracy:
                self.logger.info(f"move_abs: goal {target.round(3)*100} reached within {accuracy*100} cm.")
                break
            
            a = math.atan2(diff[1], diff[0])/math.pi*180
            if speed < 0:
                a += 180
            
            self.logger.info(f"move_abs: segment {ii:02d}: {start.round(3)*100} to {end.round(3)*100}: {a:.1f}°, {distance*100:.1f} cm.")
            await self.turn_absolute(a, np.abs(speed), accuracy=max(3,10*min(distance,1)))
            
            if distance > 1:
                self.nav.tell_straight_move_start()
            self.drive.move(min(distance,1)*self.putz_per_meter, -speed)
            
            await self.drive.finished
            self.nav.tell_straight_move_done()

            ii += 1

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
        ii_failed = 0
        while True:
            next_x = random.randint(xmin, xmax)
            next_y = random.randint(ymin, ymax)
            self.logger.info(f'move_random: Random step to {next_x}, {next_y}, speed: {speed}')
            try:
                await self.move_absolute(next_x, next_y, speed, accuracy=40, evade=False)
                ii_failed = 0
            except PathForbiddenError as err:
                if ii_failed > 100:
                    self.logger.warning('move_random: Tried 100 random moves, all were forbidden. Abandoning.')
                    raise err
                else:
                    ii_failed += 1
                    self.logger.debug(f'move_random: Random move is forbidden: {err}. Trying another...')
            
    async def move_circle(self, speed=60, stride=1, exit_x=None, exit_y=None):
        speed = int(speed)
        N_steps = len(self.config.waypoint_x)
        waypoints = np.stack([self.config.waypoint_x, self.config.waypoint_y]).T/100
        distances = ((self.nav.get_position().reshape(1,2) - waypoints)**2).sum(axis=1)**.5
        cur_step = np.argmin(distances)
        self.logger.info(f'move_circle: starting with {N_steps} waypoints. Distances are {distances.round(3)*100} cm; closest point is {cur_step}')
        while True:
            self.logger.info(f'move_circle: to circle step {cur_step} at {waypoints[cur_step,:]}')
            await self.move_absolute(100*waypoints[cur_step, 0], 100*waypoints[cur_step, 1], 
                speed=speed, accuracy=40, evade=False)
            cur_step = (cur_step + stride) % N_steps
            if (exit_x is not None) and (exit_y is not None):
                pos = self.nav.get_position()
                if not self.keepout.is_line_forbidden(pos[0], pos[1], exit_x/100., exit_y/100.):
                    self.logger.info(f'move_circle: path to exit position {exit_x}, {exit_y} is open. Moving there and stopping.')
                    await self.move_absolute(exit_x, exit_y, speed=speed, accuracy=10, evade=False)
                    break
            
    async def move_straight(self, distance=0, speed=60, xmin=None, xmax=None, ymin=None, ymax=None):
        #TODO CATCH INVALID END COORDINATE
        distance, xmin, xmax, ymin, ymax = ((int(p) / 100 if p is not None else None) for p in (distance, xmin, xmax, ymin, ymax))
        dist_putz = self.putz_per_meter*distance
        ini_pos = self.nav.get_position()
        final_pos = ini_pos + [np.cos(self.nav.get_angle()*np.pi/180)*distance, np.sin(self.nav.get_angle()*np.pi/180)*distance]
        speed = np.abs(int(speed))

        if (xmin is not None and (final_pos[0] < xmin)) or (xmax is not None and (final_pos[0] > xmax)) \
                or (ymin is not None and (final_pos[1] < ymin)) or (ymax is not None and (final_pos[1] > ymax)):
            raise PathForbiddenError(f'Straight move beyond defined bounds', ini_pos, final_pos)

        self.keepout.validate(ini_pos[0], ini_pos[1], final_pos[0], final_pos[1])
        self.logger.info(f'move_straight: Straight move by {round(distance, 3)*100} cm = {dist_putz:.3f} putz. Estimated final position is {final_pos}.')
        self.drive.move(dist_putz, speed=-speed)
        await self.drive.finished
        self.logger.info(f'move_straight: finished. Actual pos is {self.nav.get_position()} -> {self.nav.get_position() - final_pos} off ({((self.nav.get_position() - final_pos)**2).sum()**.5:.3f} m).')

    async def move_back_forth(self, range=0, max_angle=15, speed=60, random=1, xmin=None, xmax=None, ymin=None, ymax=None):
        curr_pos_linear = 0
        new_d = 0
        start_pos = self.nav.get_position()
        start_angle = self.nav.get_angle()
        ii_failed = 0
        self.logger.info(f'move_back_forth: started at angle {start_angle:.1f}, range {range}, speed {speed}, max dev. {max_angle}')
        while True:
            a = ((self.nav.get_angle() - start_angle) + 180) % 360 - 180
            self.logger.debug(f'move_back_forth: angle deviation is now {a:.1f}')
            if abs(a) > max_angle:
                await self.turn_absolute(start_angle, speed=50, accuracy=max_angle/2, slow_angle=max_angle)
            if new_d >= 0:
                new_d = np.random.randint(-range//2-curr_pos_linear, 0) if random else -range//2
            else:
                new_d = np.random.randint(1, range//2+1-curr_pos_linear) if random else range//2
            try:
                await self.move_straight(distance=new_d, speed=speed, xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)
                ii_failed = 0
            except PathForbiddenError as err:
                if random:
                    if ii_failed > 100:
                        self.logger.warning('Tried 100 random B-F moves, all were impossible. Abandoning.')
                        raise err
                    else:
                        ii_failed += 1
                        self.logger.debug(f'Random B-F move is forbidden: {err}. Trying another...')
                else:
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
    #TODO why is this not a method of Putzini?
    move_task = asyncio.Future()
    async for message in messages:

        try:
            cmd = json.loads(message.payload.decode("utf-8"))                    
        except Exception as e:
            logger.exception(f"parse_json_commands: Error parsing {message.payload.decode('utf-8')}")
            cmd = {}

        try:
            cmd = {k: v for k, v in cmd.items() if v is not None} # TODO see if this really works
            putzini.command_state.update(cmd)

            logger.debug(f"parse_json_commands: Executing: {cmd}")
            # logger.info(f"parse_json_commands: Global state is {putzini.command_state}")

            if "lamp" in cmd and cmd["lamp"] != None:
                l = cmd["lamp"]
                putzini.lamp.set_lamp(l)
                logger.info(f"parse_json_commands: setting lamp to: {l}")

            if "head" in cmd and cmd["head"] != None:
                try:
                    h = int(cmd["head"])
                except ValueError as err:
                    if isinstance(cmd['head'], str):
                        logger.info(f"parse_json_commands: Received Head Command:", cmd['head'])
                    else:
                        raise err
                else:
                    putzini.lamp.set_head(h)
                    logger.info(f"parse_json_commands: setting head to: {h}")

            if "vacuum" in cmd and cmd["vacuum"] != None:
                v = int(cmd["vacuum"])
                putzini.neck.set_vacuum(v)
                logger.info(f"parse_json_commands: switching vacuum cleaner {'on' if v==1 else 'off'}")

            if "height" in cmd and cmd["height"] != None:
                h = int(cmd["height"])
                if "vacuum" in cmd:
                    # minimum required inter-call time for neck and vacuum calls
                    await asyncio.sleep(10e-3)
                putzini.neck.set_position(h)
                logger.info(f"parse_json_commands: setting neck to {h}")

            if 'audio' in cmd and cmd["audio"] != None:
                acmd = cmd["audio"]
                if isinstance(acmd, str) and (acmd.lower() == 'stop'):
                    putzini.sound.stop()
                else:
                    fn = os.path.join('/home/putzini/audio', acmd['folder'], acmd['file'])
                    logger.info("parse_json_commands: Triggering audio file %s at volume %s as %s", fn, acmd['vol'], ('loop' if acmd['loop'] else 'one-shot'))
                    asyncio.ensure_future(putzini.sound.play(fn, bool(acmd['loop'] ), acmd['vol']))

            if "move" in cmd and cmd["move"] != None:
                logger.info('parse_json_commands: Executing drive task: %s', cmd["move"])
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
                    # print(pp)
                elif cmd["move"].startswith("moveToAngle"):
                    pp = parse_command("moveToAngle", 2, cmd["move"])
                    # print(pp)
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
                else:
                    logger.error("parse_json_commands: Unknown command: %s", cmd["move"])
                
                move_task.add_done_callback(putzini.state.set_idle)       

        except Exception as e:
            # print(e)
            logger.exception(f"parse_json_commands: Error executing {message.payload.decode('utf-8')}:")


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
        
        # manager = client.filtered_messages("putzini/turn_absolute")
        # messages = await stack.enter_async_context(manager)
        # await client.subscribe("putzini/turn_absolute")
        # tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.turn_absolute)))
        
        # manager = client.filtered_messages("putzini/move_absolute")
        # messages = await stack.enter_async_context(manager)
        # await client.subscribe("putzini/move_absolute")
        # tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.move_absolute)))

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
