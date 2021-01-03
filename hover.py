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
            
            #speed_l = int(speed_l * 1.05)
        
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


class PutziniNav:
    def __init__(self, mqtt_client):
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
        self.proc = await asyncio.create_subprocess_exec(cmd,'live:0','calib_usbgs/map.yml','calib_usbgs/usbgs.yml','-f ','arucoConfig.yml','-r','0', stdout=asyncio.subprocess.PIPE)
        asyncio.ensure_future(self._reader_task())

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

                self.timestamp = time.time()
            
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
                    print(f'WARNING: out of plane angles {alpha[:2]} exceed limit.')
                else:
                    self.position = self.RT_rp[:3,-1]
                    self.alpha = alpha
                asyncio.ensure_future(self.mqtt_client.publish("putzini/position",repr(self.RT_rp),qos=0))  
                

    # def zero_here(self): # sets position and Z-angle of c and m coordinate systems equal

    def get_angle(self):
        return self.alpha[2]

    async def get_new_angle(self):
        print(self.timestamp, self.t_last_angle)
        while self.timestamp == self.t_last_angle:
            print(self.timestamp)

            await asyncio.sleep(0.2)
        self.t_last_angle = self.timestamp
        return self.alpha[2]
    
    def get_position(self, wait_for_new=False):
        return self.position[:2]

    def set_reference_position(self, clear=False):    
        if clear:
            self.opts['marker-system'] = {'x': 0, 'y': 0, 
                'z': 0, 'angle': 0}
        else:
            RT_calib = np.matmul(np.linalg.inv(self.RT_pm), np.diag([1, -1, -1, 1]))
            angles = transform.Rotation.from_dcm(RT_calib[:3,:3]).as_euler('XYZ')*180/np.pi

            self.opts['marker-system'] = {'x': float(RT_calib[0,-1]), 'y': float(RT_calib[1,-1]), 
                'z': float(RT_calib[2,-1]), 'angle': float(angles[2])}

        self.init_reference_system()

        with open('putziniNav.yml', 'w') as fh:
            yaml.dump(self.opts, fh)

class Putzini:
    def __init__(self, mqtt_client):
        self.drive = PutziniDrive(mqtt_client)
        self.nav = PutziniNav(mqtt_client)
        self.lamp = PutziniLamp()
        self.neck = PutziniNeckAndVacuum()
        
        self.putz_per_degree = 20
        self.putz_per_degree_array = np.ones(10)*self.putz_per_degree
        
        self.putz_per_meter = 17241*0.9

    async def start(self):
        d = asyncio.ensure_future(self.drive.connect())
        n = asyncio.ensure_future(self.nav.start())
        l = asyncio.ensure_future(self.lamp.connect())
        m = asyncio.ensure_future(self.neck.connect())
        
        await asyncio.gather(d, n, l, m)
    
    async def turn_absolute(self, angle, speed=60, accuracy=2, slow_angle=20):
        angle = int(angle)

        print(f'Turn to {angle} from {self.nav.get_angle()}, speed {speed}, acc. {accuracy}, slowdown below {slow_angle}')
        while True:
            # if needed wrap into loop
            
            # old_angle = self.nav.get_angle()
            old_angle = (await asyncio.gather(self.nav.get_new_angle()))[0]
            
            # calculate the needed relative turn 
            # https://stackoverflow.com/questions/1878907
            a = angle - old_angle
            a = (a + 180) % 360 - 180

            print (f"{old_angle:.2f} to {angle:.2f} => delta={a:.2f}; act spd=[{self.drive.meas_speed_r:.1f}, {self.drive.meas_speed_l:.1f}]; spd={speed}")
            
            if (abs(a) < accuracy) and (self.drive.meas_speed_r < 5) and (self.drive.meas_speed_l < 5):
                self.drive.stop() #not working? no idea why!
                break
            
            if abs(a) < slow_angle:
                speed = min(speed,50)
                
            # turn
            distance = a*self.putz_per_degree
            self.drive.turn(distance, speed)
            await asyncio.sleep(200e-3)       

    async def turn_relative(self, delta_angle, speed=60, accuracy=2, slow_angle=20):

        # delta_angle = int(delta_angle)
        await self.turn_absolute(delta_angle + self.nav.get_angle(), speed=speed, accuracy=accuracy, slow_angle=slow_angle)

        
    async def move_absolute(self, x, y=0, speed=60):
        x= int(x) / 100
        y= int(y) / 100
        
        while True: 
            start = self.nav.get_position()
            end = np.array([x,y])

            diff = end-start
            distance = np.linalg.norm(diff)
            
            if distance < 0.1:
                break
            
            a = math.atan2(diff[1], diff[0])/math.pi*180
            
            print (f"move from {start} to {end}: turn to {a}° and drive {distance}m.")
            await self.turn_absolute(a, speed)
            
            self.drive.move(min(distance,1)*self.putz_per_meter, -speed)
            await self.drive.finished
            
    async def move_random(self, xmin=0, xmax=0, ymin=0, ymax=0, speed=60):
        while True:
            next_x = random.randint(xmin, xmax)
            next_y = random.randint(ymin, ymax)
            print(f'Random step to {next_x}, {next_y}, speed: {speed}')
            await self.move_absolute(next_x, next_y, speed)
            
    async def move_straight(self, distance=0, speed=60, xmin=None, xmax=None, ymin=None, ymax=None):
        raise NotImplementedError('do it')

    async def move_back_forth(self, range=0, speed=60, random=1):
        raise NotImplementedError('do it')

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


async def parse_json_commands(messages, putzini):
    move_task = asyncio.Future()
    async for message in messages:
        try:
            cmd = json.loads(message.payload.decode("utf-8"))
            print (f"cmd: {cmd}")
            if "lamp" in cmd and cmd["lamp"] != None:
                putzini.lamp.set_lamp(cmd["lamp"])
                print(f"setting lampe to: {l}")
            if "head" in cmd and cmd["head"] != None:
                putzini.lamp.set_head(int(cmd["head"]))
                print(f"setting head to: {h}")
            if "move" in cmd and cmd["move"] != None:
                if cmd["move"] == "stop()":
                    move_task.cancel()
                    putzini.drive.stop()
                elif cmd["move"].startswith("moveToPos"):
                    pp = parse_command("moveToPos", 3, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    move_task = asyncio.ensure_future(putzini.move_absolute(pp[0],pp[1],pp[2]))
                elif cmd["move"].startswith("moveToAngle"):
                    pp = parse_command("moveToAngle", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    move_task = asyncio.ensure_future(putzini.turn_absolute(pp[0],pp[1]))
                elif cmd["move"].startswith("moveByAngle"):
                    pp = parse_command("moveByAngle", 2, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    move_task = asyncio.ensure_future(putzini.turn_relative(pp[0],pp[1]))                    
                elif cmd["move"].startswith("moveRandom"):
                    pp = parse_command("moveRandom", 5, cmd["move"])
                    move_task.cancel()
                    putzini.drive.stop()
                    move_task = asyncio.ensure_future(putzini.move_random(*pp))
                elif cmd["move"].startswith("setReferencePos"):
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.nav.set_reference_position(clear=False)         
                elif cmd["move"].startswith("clearReferencePos"):
                    move_task.cancel()
                    putzini.drive.stop()
                    putzini.nav.set_reference_position(clear=True)                                
        except Exception as e:
            print(e)
            print(f"error parsing {message.payload.decode('utf-8')}")


async def main():

    client = mqtt.Client("localhost")
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
        tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.lamp.set_head)))
        
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
    loop.run_until_complete(main())
    loop.close()
