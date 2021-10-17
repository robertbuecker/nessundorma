#!/usr/bin/env python3

import asyncio
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
from putzini_config import PutziniConfig
from putzini_cam import PutziniCam
from putzini_keepout import PutziniKeepoutArea, KeepoutError
from putzini_state import PutziniState
from putzini_nav import PutziniNav2
from typing import Optional
import logging
from collections import namedtuple

logger = logging.getLogger('putzini_calib')
CalibPos = namedtuple('PutziniCalibPos', ['r_act', 'r_calc', 'distances', 'weights'])

async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    await client.connect()
    state = PutziniState(client)
    keepout = PutziniKeepoutArea(client, config)
    nav = PutziniNav2(client, state, config, keepout)
    logger.info('Connecting navigation system...')
    await nav.connect()
    position_list = []    
    while True:
        try:
            pos_str = input('Please enter Putzini position in cm as x, y or empty to finish: ')
            if len(pos_str) == 0:
                break
            else:
                pos_xy = pos_str.split(',')
                assert len(pos_xy) == 2
                pos_xy = np.array([float(pos_xy[0].strip())/100, float(pos_xy[1].strip())/100])
            print(f'{nav.N_valid} valid position readouts within {config.nav_avg_time} s averaging time.')
            new_pos = CalibPos(r_act=pos_xy, r_calc=nav.get_position(), 
                               distances=nav.distances, weights=nav.anchor_weights)
            print(f'Have new position {new_pos}')
            position_list.append(new_pos)
            
        except Exception as err:
            print(f'An error occurred: {err}')
            cont = input('Continue (y/n)')
            if not 'y' in cont:
                raise err         
            
    print(f'Stopped. Calibration data taken from {len(position_list)} points:')
    print('\n'.join(position_list))
    
    # variable to optimize (K: # calib points, N: # markers):
    K, N = len(position_list), nav.anchor_pos.shape[0]
    # collect and reshape as broadcastable to K x N x 2
    anchor_positions = nav.anchor_pos[:,:2].reshape((1, N, 2)) # N x 2
    calc_pos = np.stack([p.r_calc for p in position_list]).reshape((K, 1, 2)) # K x 2
    distances = np.stack([p.distances for p in position_list]).reshape((K, N, 1)) # K x N
    putzini_locs = np.stack([p.r_act for p in position_list]).reshape((K, 1, 2)) # K x 2
    weights = np.stack([p.weights for p in position_list]).reshape((K, N, 1)) # N
    
    np.savez('calibdata.npz',
             anchor_positions=anchor_positions, calc_pos=calc_pos, distances=distances, 
             putzini_locs=putzini_locs, weights=weights)
 
    print(f'Initial position deviations (cm):')
    print(((calc_pos-putzini_locs).squeeze()*100).round().astype(int))
    
    def error_inverse(x):
        # solve for anchor positions
        dist_err = ((x - putzini_locs)**2).sum(axis=2, keepdims=True)**.5 - distances
        f = np.nansum(weights * dist_err**2, keepdims=False)
        return f
    
    anchor_positions = minimize(error_inverse, anchor_positions, method='BFGS').x
    
    def error_forward(x):
        # solve for Putzini position
        dist_err = ((anchor_positions - x)**2).sum(axis=2, keepdims=True)**.5 - distances
        f = np.nansum(weights * dist_err**2, keepdims=False)
        return f    
    
    new_calc_pos = minimize(error_forward, putzini_lo, method='BFGS').x
    
    print(f'Final position deviations (cm):')
    print(((new_calc_pos-putzini_locs).squeeze()*100).round().astype(int))
    
    print('Calculated anchor positions are:\n-----')
    print(f'anchor_x: {list((anchor_positions[:,0]*100).round().astype(int))}')
    print(f'anchor_y: {list((anchor_positions[:,1]*100).round().astype(int))}')
    print('-----')
    print('Please copy (add/overwrite) these lines into putzini.yaml')
    
    await nav.stop_ranging()
    await asyncio.sleep(1)
    await client.disconnect()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    loop.run_until_complete(main())
    loop.close()