
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

import json
from sys import argv
import time
import numpy as np
import time
from scipy.optimize import minimize
import board
import adafruit_bno055
from putzini_config import PutziniConfig
from putzini_cam import PutziniCam
from putzini_keepout import PutziniKeepoutArea, KeepoutError
from putzini_state import PutziniState
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class PutziniNav2:
    # Anchor-based system

    def __init__(self, mqtt_client, putzini_state: PutziniState, 
                putzini_config: PutziniConfig, 
                putzini_keepout: PutziniKeepoutArea, 
                putzini_cam: Optional[PutziniCam] = None):
        # print('Position class started')
        # asyncio.ensure_future(self.connect())
        self.initialized = False
        self.mqtt_client = mqtt_client

        self.state = putzini_state
        self.config = putzini_config
        self.keepout = putzini_keepout
        self.cam = putzini_cam
        self.logger = logger

        # self.anchor_idx = {b'B4DE': 0, b'B4D3': 1, b'B4D9': 2}
        self.anchors = putzini_config.anchor_names
        self.tag = putzini_config.tag_name
        self.anchor_idx = {name.encode(): ii for ii, name in enumerate(self.anchors)}
        print(self.anchor_idx)

        # self.anchor_pos = np.array([[400, -260, 0],
        #                    [400,+400, 0],
        #                    [+35,0, 0]],dtype=float)/100

        self.anchor_pos = np.array(
            [putzini_config.anchor_x,
            putzini_config.anchor_y,
            [0]*len(putzini_config.anchor_x)]
        ).T/100.

        self.distances = np.array([0.]*len(self.anchors))
        self.distances_sig = np.array([0.]*len(self.anchors))
        self._distance_buffer = {k.encode(): [] for k in putzini_config.anchor_names}
        self._alpha_buffer = [np.array([0., 0., 0.])]
        self.N_valid = {k.encode(): 0 for k in putzini_config.anchor_names}
        self.position = self.anchor_pos.mean(axis=0)
        self.position[2] = -0.05
        self.RT_rp = np.eye(4)
        self.sensor = None
        self.alpha = np.array([0.]*3)
        self.t_last_angle = 0
        self.timestamp = -1.
        self.avg_len = 20
        self.t_update = 1000/putzini_config.nav_update_rate
        self.sensordat = {}
        self.calibration = putzini_config.bno055_calib
        self.calibration = {}
        self.calibrated = (0,0,0,0)
        self.moving_straight = False
        self.straight_move_buffer = [np.empty(3)]

    async def connect(self, url='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', baudrate=512000):

        # Position sensor
        t0 = time.time()
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)  
        print(f'{1e3*(time.time()-t0)} ms:', 'Positioning device connected.')
        asyncio.ensure_future(self._reader_task())
        await self.start_ranging()
        print(f'{1e3*(time.time()-t0)} ms:', 'Positioning started.')

        # Orientation sensor
        print(f'{1e3*(time.time()-t0)} ms:', 'Starting up orientation sensor.')
        i2c = board.I2C()
        i2c.init(board.SCL_1,board.SDA_1, 800)
        print(f'{1e3*(time.time()-t0)} ms:', 'I2C initialized.')
        self.sensor = adafruit_bno055.BNO055_I2C(i2c) # TODO: Why does this take so long here?
        print(f'{1e3*(time.time()-t0)} ms:', 'Orientation sensor connected. Reading initial calibrations from sensor.')
        self.read_calibration_from_sensor()
        print(f'{1e3*(time.time()-t0)} ms:', 'Sending stored calibrations to sensor.')
        await self.write_calibration_to_sensor(reset=True, restart=False)
        # asyncio.ensure_future(self._read_bno055())
        print(f'{1e3*(time.time()-t0)} ms:', 'Orientation sensor started.')    

    def read_calibration_from_sensor(self):
        self.calibration = {'off_acc': self.sensor.offsets_accelerometer,
                            'off_gyr': self.sensor.offsets_gyroscope,
                            'off_mag': self.sensor.offsets_magnetometer,
                            'rad_mag': self.sensor.radius_magnetometer,
                            'rad_acc': self.sensor.radius_accelerometer}
        asyncio.ensure_future(self.mqtt_client.publish("putzini/calibration", json.dumps(self.calibration)))
        asyncio.ensure_future(self.mqtt_client.publish("putzini/calibrated", json.dumps(self.calibrated)))

    async def write_calibration_to_sensor(self, reset=True, restart=True):
        reset = int(reset) # if sent over mqtt
        calib = self.calibration
        if reset and (self.config.bno055_calib is not None):
            calib.update(self.config.bno055_calib)
        elif reset:
            print('Cannot reset BNO055 parameters, as there are none in putzini.yaml')
        t0 = time.time()
        if restart:
            # self.sensor._write_register(adafruit_bno055._TRIGGER_REGISTER, 0x20 | 
                # self.sensor._read_register(adafruit_bno055._TRIGGER_REGISTER)) 
            self.sensor._write_register(adafruit_bno055._TRIGGER_REGISTER, 0x20) 
            await asyncio.sleep(.8)
            print(f'Sensor restart completed after {1000*(time.time() - t0)} ms')
        self.sensor.mode = adafruit_bno055.CONFIG_MODE
        self.sensor.offsets_accelerometer = calib['off_acc']
        self.sensor.offsets_gyroscope = calib['off_gyr']
        self.sensor.offsets_magnetometer = calib['off_mag']
        self.sensor.radius_accelerometer = calib['rad_acc']
        self.sensor.radius_magnetometer = calib['rad_mag']
        # self.sensor.mode = adafruit_bno055.NDOF_FMC_OFF_MODE
        print(f'Sensor parameter write completed after {1000*(time.time() - t0)} ms')
        self.sensor.mode = adafruit_bno055.NDOF_MODE     
        await asyncio.sleep(1)
        self.read_calibration_from_sensor()   
        print(f'Sensor ready after {1000*(time.time() - t0)} ms')
            
    async def start_ranging(self):
        await self.stop_ranging()
        self.writer.write(b'$PL,\r\n')
        # config_string = f'$PK,{self.ids["anchor_1"]},2,1,{self.ids["anchor_2"]},{self.ids["anchor_3"]},{self.ids["tag"]},\r\n'
        config_string = f'$PK,{self.tag},0,{len(self.anchors)},{",".join(self.anchors)},\r\n'
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

            self.timestamp = time.time()                
            
            # self.alpha from angle buffer

            for k, v in self._distance_buffer.items():
                # print(v)
                self.N_valid[k] =  len(v) - self.config.nav_avg_len
                if len(v) > self.config.nav_avg_len:
                    v = v[-self.config.nav_avg_len:]
                self.distances[self.anchor_idx[k]] = np.mean(v)
                dist_std = np.std(v)
                self._distance_buffer[k] = v
                t0 = time.time()

                # weights for optimization
                try:
                    ko_len = np.array([self.keepout.N_line_keepout(self.position[0], self.position[1],
                        self.anchor_pos[ii,0], self.anchor_pos[ii,1]) for ii in range(len(self.anchor_idx))])
                    w = 1/(self.distances +  ko_len**2)
                        # print(self.distances, ko_len)
                except Exception as err:
                    print(f'Cannot calculate weights: {err}')
                    w = np.array([1.]*len(self.distances))

            # TODO tidy up this mess!!!
            if self.cam is None:
                await self._read_bno055()
                if len(self._alpha_buffer):
                    self.alpha = self._alpha_buffer[-1]
                    self.alpha = ((-self.alpha[0] - self.config.room_rotation + 180) % 360 - 180, self.alpha[1], self.alpha[2])      
                    N_alpha_valid = 1
                else:
                    N_alpha_valid = 0

                self._alpha_buffer = []            
                self.logger.info('Angle from BNO055 is %s w.r.t. room CS.', self.alpha[0])

            else:
                N_alpha_valid = 1
                alpha_cam = self.cam.get_angle()
                alpha_room = (alpha_cam - self.config.room_rotation - self.config.cam_rotation + 180) % 360 - 180
                self.logger.info('Angle from camera is %s raw, %s w.r.t. room CS.', alpha_cam, alpha_room)
                self.alpha = np.array([alpha_room, 0, 0])

            tw = time.time() - t0
            include_z = False

            if include_z:
                def error(x):
                    dist_err = ((self.anchor_pos - x.reshape(1,3))**2).sum(axis=1)**.5 - self.distances
                    f = (w * dist_err**2).sum()
                    return f
                self.position = minimize(error, self.position, method='BFGS').x

            else:
                def error(x):
                    dist_err = ((self.anchor_pos[:,:2] - x[:2].reshape(1,2))**2).sum(axis=1)**.5 - self.distances
                    f = (w * dist_err**2).sum()
                    return f
                self.position[:2] = minimize(error, self.position[:2], method='BFGS').x        

            try:
                self.keepout.validate(self.position[0], self.position[1])
            except KeepoutError as err:
                print(err)

            # self.position = pos_solve(self.distances, self.anchor_pos, self.position)/100.
            # print(f'N = {N_valid}; d = {(self.distances*100).round(1)} cm; x = {(self.position*100).round(1)} cm; tOpt = {(time.time()-t0)*1000:.0f} ms')
            # print(f'tW = {tw*1000} ms, tOpt = {(time.time()-t0)*1000:.0f} ms')
            # dirty fix: just inverting in-plane angle for now
            c, s = np.cos(self.alpha[0]/180*np.pi), np.sin(self.alpha[0]/180*np.pi)
            self.RT_rp = np.array([[c,-s,0,self.position[0]], 
                                    [s,c,0,self.position[1]], 
                                    [0,0,1,self.position[2]], 
                                    [0,0,1,0]])

            if self.moving_straight:
                self.straight_move_buffer.append(np.concatenate([self.position[:2], self.alpha[:1]]))

            # print(self.N_valid)
            asyncio.ensure_future(self.mqtt_client.publish("putzini/distances", 
                    json.dumps({'N': list(self.N_valid.values()) + [N_alpha_valid], 'd': self.distances.round(4).tolist(), 'w': w.round(4).tolist()}),
                    # f'{{"N": {N_valid}, "d": {self.distances.round(4)}}}', 
                    qos=0))
            asyncio.ensure_future(self.mqtt_client.publish("putzini/position", repr(self.RT_rp), qos=0))
            self.state.set_position_with_alpha(self.position, self.alpha)

            # try:
            #     self.sensordat = {
            #             'B': self.sensor.magnetic,
            #             'aA': self.sensor.gyro,
            #             'aL': self.sensor.linear_acceleration,
            #             'g': self.sensor.gravity,
            #             'a': self.sensor.acceleration}
            #     asyncio.ensure_future(self.mqtt_client.publish("putzini/sensordata", json.dumps(self.sensordat)))

            # except OSError as err:
            #     print(f'Could not read orientation sensor: {err}. Maybe it is restarting?')                        

    async def _read_bno055(self):
        # while True:
        try:
            euler = np.array(self.sensor.euler)
            if not np.isnan(euler).any():
                self._alpha_buffer.append(euler)

            cstat = self.sensor.calibration_status

            if self.calibrated != cstat:
                self.calibrated = cstat
                self.read_calibration_from_sensor()   
            
                if cstat[3] < self.config.minimum_calib_level:
                    print('Mag calib off. Reloading from file.')
                    asyncio.ensure_future(self.mqtt_client.publish('putzini/error', "Sensor calib off. Reloading from file."))
                    await self.write_calibration_to_sensor(reset=True, restart=True)

        except (OSError, TypeError) as err:
            print(f'Could not read orientation sensor: {err}. Maybe it is restarting?')
            await asyncio.sleep(0.5)

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
                    if (not d1 == 0) and (not d1 >= self.config.max_distance):
                        self._distance_buffer[tag_id].append(float(d1)/100.)
                        # new_dist[self.anchor_idx[tag_id]] = float(d1)/100.
                    # self._distance_buffer.append(new_dist)
                except Exception as err:
                    print(f'Could not decode distance message: {par}')
                    # pass
                    raise err
                                            
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
        while self.timestamp <= self.t_last_angle:
            await asyncio.sleep(0.01)
        self.t_last_angle = self.timestamp
        return self.alpha[0]
    
    def get_position(self):
        return self.position[:2]

    def store_calib(self, *args):
        print('Extra args:', args)
        self.config.bno055_calib = self.calibration
        print('Storing calibration:', self.calibration)
        self.config.to_yaml()

    def tell_straight_move_start(self):
        self.moving_straight = True
        self.straight_move_buffer = [np.empty(3)]

    def tell_straight_move_done(self):
        self.moving_straight = False
        buffer = np.stack(self.straight_move_buffer)
        trajectory = buffer[:,:2]
        d_trajectory = np.diff(trajectory, axis=0)
        angle_trajectory = np.arctan2(d_trajectory[:,1], d_trajectory[:,0]) * 180/np.pi
        angle_sensor = (buffer[:-1,2] + buffer[1:,2])/2
        d_angle = (angle_trajectory - angle_sensor + 180) % 360 - 180
        angle_dev = np.mean(d_angle[len(d_angle)//4:])
        angle_dev_std = np.std(d_angle[len(d_angle)//4:])
        angle_dev_med = np.median(d_angle[len(d_angle)//4:])
        print(angle_trajectory, angle_sensor)
        print(f'Angle deviation is {angle_dev}, SD {angle_dev_std}, Median {angle_dev_med}.')
        # d_trajectory = median_filter()
        