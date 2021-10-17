#!/usr/bin/env python3

import asyncio
import numpy as np
from scipy.spatial import transform
import yaml
from sys import argv
import os
import time
import logging

import numpy as np

logger = logging.getLogger(__name__)

class PutziniCam:
    def __init__(self, mqtt_client, putzini_config):
        self.config = putzini_config
        self.logger = logger 
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
        self.position = self.RT_rp[:3,-1]
        self.alpha = np.array([0., 0., 0.])
        self.timestamp = time.time()       

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
            self.logger.info('Setting reference system w.r.t. marker system to \n%s', reference)
            # print(reference)
            self.RT_mr = np.matmul(reference, np.diag([1, -1, -1, 1]))

        else:
            self.RT_mr = np.diag([1, -1, -1, 1])

        # cam on Putzini is hard-coded: rotated by about 90 deg and 15 cm above wheel hubs
        self.RT_pc = np.array([[0,1,0,0], [-1,0,0,0], 
                        [0,0,1,0.15], [0,0,0,1]])             

    async def start(self):
        cmd = 'aruco_dcf_mm' if len(argv) > 1 and argv[1] == 'gui' else 'aruco_dcf_mm_nogui'
        self.proc = await asyncio.create_subprocess_exec(cmd,'live:0',self.config.marker_map,'calib_usbgs/usbgs.yml','-f ','arucoConfig.yml','-r','0', stdout=asyncio.subprocess.PIPE)
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
                    wstr = f'Out of plane angles {alpha[:2]} exceed limit.'
                    # asyncio.ensure_future(self.mqtt_client.publish("putzini/state",json.dumps({'navstatus': wstr}),qos=0))  
                    self.logger.warning(wstr)
                else:
                    self.position = self.RT_rp[:3,-1]
                    self.alpha = alpha
                    self.timestamp = time.time()
                    self.logger.debug('Have raw cam data: %s deg; %s pos', self.alpha.round(1), self.position.round(2))
                    # asyncio.ensure_future(self.mqtt_client.publish("putzini/state",json.dumps({'navstatus': 'OK'}),qos=0))  

                # asyncio.ensure_future(self.mqtt_client.publish("putzini/position",repr(self.RT_rp),qos=0))  

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

async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    t0 = time.time()
    cam = PutziniCam(client)
    await cam.start()
    # p = lambda txt: print(f'(round(time.time()-t0,2))', txt)
    while True:
        logger.info('%s markers. Position is %s, angle is %s', cam.detected, cam.get_position(), cam.get_angle())
        await asyncio.sleep(1)    

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()