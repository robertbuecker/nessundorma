#!/usr/bin/env python3

import struct
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

        # self.anchor_pos = np.array([[400, -260, 0],
        #                    [400,+400, 0],
        #                    [+35,0, 0]],dtype=float)/100

        self.anchor_pos = np.array(
            [putzini_config.anchor_x,
            putzini_config.anchor_y,
            [0]*len(putzini_config.anchor_x)]
        ).T/100.

        self.logger.info('Anchors in configuration file are %s at positions:\n %s', self.anchor_idx, self.anchor_pos)

        self.distances = np.array([0.]*len(self.anchors))
        self.distances_sig = np.array([0.]*len(self.anchors))
        self.anchor_weights = np.array([0.]*len(self.anchors))
        self._distance_buffer = {k.encode(): [] for k in putzini_config.anchor_names}
        self._alpha_buffer = [np.array([0., 0., 0.])]
        self.N_valid = {k.encode(): 0 for k in putzini_config.anchor_names}
        self.distance_factors = {k.encode(): fac for k, fac in zip(putzini_config.anchor_names, putzini_config.distance_factors)}
        # self.position = self.anchor_pos.mean(axis=0)
        self.position = np.array([3, 0., 0.])
        self.position[2] = -0.05
        self.RT_rp = np.eye(4)
        self.sensor = None
        self.sensor_cam_offset = 0.
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
        self._ranging_task = None
        self._update_task = None

    async def connect(self, url='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 
        baudrate=512000):

        # Position sensor
        t0 = time.time()
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)  
        self.logger.info('connect: Positioning device connected.')
        await self.start_ranging()

        # Orientation sensor
        if self.config.use_bno055:
            self.logger.info('connect: Starting up orientation sensor.')
            i2c = board.I2C()
            i2c.init(board.SCL_1,board.SDA_1, 800)
            self.logger.info('connect: I2C initialized.')
            self.sensor = adafruit_bno055.BNO055_I2C(i2c) # TODO: Why does this take so long here?
            self.logger.info('connect: Orientation sensor connected. Reading initial calibrations from sensor.')
            self.read_calibration_from_sensor()
            self.logger.info('connect: Sending stored calibrations to sensor.')
            await self.write_calibration_to_sensor(reset=True, restart=False)
            # asyncio.ensure_future(self._read_bno055())
            self.logger.info('connect: Orientation sensor started.')    
            if self.cam is not None:
                self.logger.info('Camera is used for absolute orientation. Setting BNO055 to IMU mode')
                self.sensor.mode = adafruit_bno055.IMUPLUS_MODE
        else:
            self.logger.info('connect: Orientation sensor is disabled.')

    def read_calibration_from_sensor(self):
        if self.sensor is not None:
            self.calibration = {'off_acc': self.sensor.offsets_accelerometer,
                                'off_gyr': self.sensor.offsets_gyroscope,
                                'off_mag': self.sensor.offsets_magnetometer,
                                'rad_mag': self.sensor.radius_magnetometer,
                                'rad_acc': self.sensor.radius_accelerometer}
            asyncio.ensure_future(self.mqtt_client.publish("putzini/calibration", json.dumps(self.calibration)))
            asyncio.ensure_future(self.mqtt_client.publish("putzini/calibrated", json.dumps(self.calibrated)))

    async def write_calibration_to_sensor(self, reset=True, restart=True):
        if self.sensor is not None:
            reset = int(reset) # if sent over mqtt
            calib = self.calibration
            if reset and (self.config.bno055_calib is not None):
                calib.update(self.config.bno055_calib)
            elif reset:
                self.logger.warning('write_calibration_to_sensor: Cannot reset BNO055 parameters, as there are none in putzini.yaml')
            t0 = time.time()
            if restart:
                # self.sensor._write_register(adafruit_bno055._TRIGGER_REGISTER, 0x20 | 
                    # self.sensor._read_register(adafruit_bno055._TRIGGER_REGISTER)) 
                self.sensor._write_register(adafruit_bno055._TRIGGER_REGISTER, 0x20) 
                await asyncio.sleep(.8)
                self.logger.info(f'write_calibration_to_sensor: Sensor restart completed after {1000*(time.time() - t0)} ms')
            self.sensor.mode = adafruit_bno055.CONFIG_MODE
            self.sensor.offsets_accelerometer = calib['off_acc']
            self.sensor.offsets_gyroscope = calib['off_gyr']
            self.sensor.offsets_magnetometer = calib['off_mag']
            self.sensor.radius_accelerometer = calib['rad_acc']
            self.sensor.radius_magnetometer = calib['rad_mag']
            # self.sensor.mode = adafruit_bno055.NDOF_FMC_OFF_MODE
            self.logger.info(f'write_calibration_to_sensor: Sensor parameter write completed after {1000*(time.time() - t0)} ms')
            self.sensor.mode = adafruit_bno055.NDOF_MODE if self.cam is None else adafruit_bno055.IMUPLUS_MODE
            await asyncio.sleep(1)
            self.read_calibration_from_sensor()   
            self.logger.info(f'write_calibration_to_sensor: Sensor ready after {1000*(time.time() - t0)} ms')
            
    async def start_ranging(self):
        self._ranging_task = asyncio.ensure_future(self._read_serial())
        self.writer.write(b'$PG,\r\n')
        await asyncio.sleep(0.2)
        self.writer.write(b'$PL,\r\n')
        await asyncio.sleep(0.2)
        # config_string = f'$PK,{self.ids["anchor_1"]},2,1,{self.ids["anchor_2"]},{self.ids["anchor_3"]},{self.ids["tag"]},\r\n'
        config_string = f'$PK,{self.tag},0,{len(self.anchors)},{",".join(self.anchors)},\r\n'
        config_string = config_string.encode('utf-8')
        self.logger.info('start_ranging: Sending configuration to DW1000 master: %s', config_string)
        self.writer.write(config_string)
        await asyncio.sleep(0.2)
        self.writer.write(b'$PS,\r\n')
        await asyncio.sleep(0.2)
        self._update_task = asyncio.ensure_future(self.update_position())
        
    async def stop_ranging(self):
        self._update_task.cancel()
        self.distances = np.nan * self.distances
        self.N_valid = {k: 0 for k in self.N_valid.keys()}  
        asyncio.ensure_future(self.mqtt_client.publish("putzini/distances", 
                json.dumps({'N': list(self.N_valid.values()) + [0], 'd': self.distances.round(4).tolist(), 'w': self.distances.round(4).tolist()}),
                # f'{{"N": {N_valid}, "d": {self.distances.round(4)}}}', 
                qos=0))        
        self.writer.write(b'$PG,\r\n')
        await asyncio.sleep(0.5)            
        self._ranging_task.cancel()            
            
    async def update_position(self):
        
        last_alpha_cam = np.nan
        last_alpha_sensor = np.nan

        while True:

            ela = time.time() - self.timestamp
            # print(round(ela*1000))
            dt = self.t_update/1000. - ela
            if dt < 0:
                self.logger.debug('Update position underrun by %.1f ms', -1000*dt)
            await asyncio.sleep(dt)

            self.timestamp = time.time()                

            # compute distances from anchors. Actual position calculation happens _after_ the orientation computation
            for k, v in self._distance_buffer.items():

                v_valid = [(t_dist, dist) for t_dist, dist in v if t_dist > (self.timestamp - self.config.nav_avg_time) ]
                self.N_valid[k] =  len(v_valid)
                self.distances[self.anchor_idx[k]] = np.mean([dist for _, dist in v_valid]) if len(v_valid) > 0 else np.nan
                self._distance_buffer[k] = v_valid
                t0 = time.time()

                # weights for optimization
                try:
                    ko_len = np.array([self.keepout.N_line_keepout(self.position[0], self.position[1],
                        self.anchor_pos[ii,0], self.anchor_pos[ii,1]) for ii in range(len(self.anchor_idx))])
                    w = 1/(self.distances +  ko_len**2)
                        # print(self.distances, ko_len)
                except Exception as err:
                    self.logger.exception('update_position: Cannot calculate weights')
                    w = np.array([1.]*len(self.distances))
                    
                self.anchor_weights = w

            # compute absolute orientation
            if (self.cam is None) and (self.sensor is not None):
                # get angle from BNO055
                await self._read_bno055()
                if len(self._alpha_buffer):
                    self.alpha = self._alpha_buffer[-1]
                    self.alpha = ((-self.alpha[0] - self.config.room_rotation + 180) % 360 - 180, self.alpha[1], self.alpha[2])      
                    N_alpha_valid = 1
                else:
                    N_alpha_valid = 0

                self._alpha_buffer = []            
                # self.logger.info('Angle from BNO055 is %s w.r.t. room CS.', self.alpha[0])

            elif (self.cam is not None) and (self.sensor is not None):
                # use both cam and sensor: sensor is used to get the actual angle, camera corrects drifts by a varying offset

                alpha_cam = self.cam.get_angle()
                try:
                    sensor_angle = -self.sensor.euler[0]
                    N_alpha_valid = 1
                except (OSError, TypeError) as err:
                    self.logger.warning(f'Could not read orientation sensor: {err}. Maybe it is restarting?')
                    N_alpha_valid = 0
                    await asyncio.sleep(0.5)

                d_alpha_cam = (alpha_cam - last_alpha_cam + 180) % 360 - 180
                
                # only recalibrate if deviation is large and quality of camera signal is good, that is,
                # a minimum number of detected markers and a maximum out-of-plane tilt (which usually means something is off)
                do_recalib = (abs(d_alpha_cam) > 5) and (self.cam.detected >= 2) and (np.max(np.abs(self.cam.alpha[:2])) < 20)

                if do_recalib and (abs(d_alpha_cam) > 10):
                    self.logger.warning(f'Relative angle found by camera and gyro was {d_alpha_cam:.1f} from {self.cam.detected} markers during state {self.state.action}. This might indicate trouble.')
                
                if np.isnan(last_alpha_cam) or do_recalib:
                    last_alpha_cam = alpha_cam
                    self.sensor_cam_offset = (sensor_angle - alpha_cam + 180) % 360 - 180
                    self.logger.info(f'd_alpha_cam was {d_alpha_cam:.1f}. Resetting sensor angle difference to {self.sensor_cam_offset:.1f} deg from {self.cam.detected:d} markers.')

                alpha_final = (sensor_angle - self.sensor_cam_offset + 180) % 360 - 180
                self.alpha = np.array([alpha_final, 0, 0])

                self.logger.debug(f'alpha_cam={alpha_cam:.1f}, sensor_angle={sensor_angle:.1f}, final_angle={self.alpha[0]:.1f}')

            elif self.cam is not None:
                # get angle from camera
                N_alpha_valid = 1
                alpha_cam = self.cam.get_angle()
                alpha_room = (alpha_cam - self.config.room_rotation - self.config.cam_rotation + 180) % 360 - 180
                self.logger.debug('update_position: Angle from camera is %s raw, %s w.r.t. room CS.', alpha_cam, alpha_room)
                self.alpha = np.array([alpha_room, 0, 0])

            else:
                # self.logger.error('update_position: Have neither camera nor BNO055 - CANNOT DETERMINE ANGLE')
                N_alpha_valid = 0
                self.alpha = np.array([np.nan, 0, 0])

            tw = time.time() - t0
            include_z = False

            # compute actual position from distances
            # TODO check if nansum with minimum valid values would work, and handle nan cases gracefully

            missing = np.isnan(self.distances).sum()
            
            if missing > 0:
                self.logger.warn('%s distance signals are missing', missing)

            if (len(self.distances) - missing) < 3:
                self.logger.error('Have less than three distance signals. Cannot determine position!')

            else:
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
                self.state.set_error(str(err))
                self.logger.error('%s', err)

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

        if self.sensor is not None:
            try:
                euler = np.array(self.sensor.euler)
                if not np.isnan(euler).any():
                    self._alpha_buffer.append(euler)

                cstat = self.sensor.calibration_status

                if self.calibrated != cstat:
                    self.calibrated = cstat
                    self.read_calibration_from_sensor()   
                
                    if cstat[3] < self.config.minimum_calib_level:
                        self.logger.info('Mag calib off. Reloading from file.')
                        asyncio.ensure_future(self.mqtt_client.publish('putzini/error', "Sensor calib off. Reloading from file."))
                        await self.write_calibration_to_sensor(reset=True, restart=True)

            except (OSError, TypeError) as err:
                self.logger.warning(f'Could not read orientation sensor: {err}. Maybe it is restarting?')
                await asyncio.sleep(0.5)

        else:
            self.logger.info('Trying to read out BNO055 sensor while it is not enabled. Bug?')

    async def _read_serial(self):
        msg=b''
        ii = 0
        self.logger.info('_reader_task: Positioning reader task started')
        while True:
            msg = await self.reader.readline()
            # msg = msg.strip().decode()
            # print(msg)
            try:
                cmd, par = msg.strip().split(b',',1)
            except:
                self.logger.warning('_reader_task: Failing to split message: %s', msg)
                continue
            # cmd, par = cmd.decode(), par.decode()
            # print('Received:',cmd, par)
            if cmd == b'$PX':
                self.logger.info('_reader_task: Ping received: %s', par)

            if cmd == b'$PK':
                self.logger.info('_reader_task: Configuration received: %s', par)
                for k in self.config.anchor_names:
                    all_found = True
                    if k.encode() not in par:
                        all_found = False
                        self.logger.info('_reader_task: Anchor %s not found in configuration!', k)
                if all_found:
                    self.logger.info('_reader_task: Configuration echo is consistent with Putzini config.')
                else:
                    err_msg = f'_reader_task: Received inconsistent configuration echo: {par}'
                    self.logger.error(err_msg)
                    raise ValueError(err_msg)

            elif cmd == b'$PD':
                ii += 1
                # await asyncio.sleep(0.1)
                new_dist = np.nan*np.ones(3)
                try:
                    tag_id, a1_dist, a2_dist, a3_dist, udata, _ = par.split(b',',5)
                    d1, d2, d3 = int(a1_dist, 16), int(a2_dist, 16), int(a3_dist, 16)
                    if tag_id not in self._distance_buffer.keys():
                        self.logger.warning('_reader_task: Received distance from non-configured anchor %s', tag_id)
                    if (not d1 == 0) and (not d1 >= self.config.max_distance):
                        self._distance_buffer[tag_id].append((time.time(), float(d1)/100.*self.distance_factors[tag_id]))
                        # new_dist[self.anchor_idx[tag_id]] = float(d1)/100.
                    # self._distance_buffer.append(new_dist)
                except Exception as err:
                    self.logger.warning('_reader_task: Could not decode distance message: %s', par)
                    # pass
                    raise err
                                            
            elif cmd == b'$PS':
                self.logger.info('_reader_task: Ranging started.')
                
            elif cmd == b'$PG':
                self.logger.info('_reader_task: Ranging stopped.')
                
            elif cmd == b'$PW':
                self.logger.info('_reader_task: Present devices received: %s', par)
    
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
        # d_angle = (angle_trajectory - angle_sensor + 180) % 360 - 180
        # angle_dev = np.mean(d_angle[len(d_angle)//4:])
        # angle_dev_std = np.std(d_angle[len(d_angle)//4:])
        # angle_dev_med = np.median(d_angle[len(d_angle)//4:])
        # print(angle_trajectory, angle_sensor)
        # print(f'Angle deviation is {angle_dev}, SD {angle_dev_std}, Median {angle_dev_med}.')
        # d_trajectory = median_filter()
        
async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    config.use_bno055 = False
    client = mqtt.Client(config.mqtt_broker)
    await client.connect()
    state = PutziniState(client)
    keepout = PutziniKeepoutArea(client, config)
    nav = PutziniNav2(client, state, config, keepout)
    print('Running nav for 5 s and shutting down.')
    await nav.connect()
    for ii in range(10):
        await asyncio.sleep(0.5)
        print(ii+1, '---')
        print(f'Position is {nav.get_position()}')
        print(f'Angle is {nav.get_angle()}')
        print(f'Distances are {nav.distances.round(4).tolist()}') 
        print(f'{nav.N_valid} valid position readouts within {config.nav_avg_time} s averaging time.')        
    await nav.stop_ranging()
    await asyncio.sleep(1)
    await client.disconnect()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    loop.run_until_complete(main())
    loop.close()