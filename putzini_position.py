
import asyncio
import serial_asyncio
import asyncio_mqtt as mqtt
try:
    from contextlib import AsyncExitStack, asynccontextmanager
except:
    from async_exit_stack import AsyncExitStack

import numpy as np
from numpy import array

from numpy.linalg.linalg import pinv, LinAlgError
from scipy.spatial.distance import euclidean
import time
from scipy.optimize import minimize

def pos_solve(dist, anchors, x0=(0,0)):
    
    def error(x, dist, anchors):
        dist_err = ((anchors - x.reshape(1,2))**2).sum(axis=1)**.5 - dist
        # print((dist_err**2/dist**2))
        f = (dist_err**2/dist).sum()
        return f

    if any(x0==0):
        x0 = anchors.mean(axis=0)
        
    return minimize(error, x0, args=(dist, anchors), method='BFGS').x

class PutziniPosition:
    
    def __init__(self, mqtt_client=None):
        # print('Position class started')
        # asyncio.ensure_future(self.connect())
        self.initialized = False
        
        self.ids = {'anchor_1': 'B4DE',
                    'anchor_2': 'B4D3',
                    'anchor_3': 'B4D9',
                    'tag': 'B521'}
        
        self.anchor_idx = {b'B4DE': 0, b'B4D3': 1, b'B4D9': 2}
        
        self.anchor_pos = np.array([[0,0],
                           [575,0],
                           [565,710]])
 
        self.distances = np.array([0,0,0])
        self.distances_sig = np.array([0,0,0])
        
        self._distance_buffer = []
        
        self.position = np.array([0,0,0])    
    
    async def connect(self, 
                        url='/dev/ttyUSB1', baudrate=512000):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)  

        
        print('anchor_1 anchor connected.')     
        asyncio.ensure_future(self._reader_task())
        # asyncio.ensure_future(self._writer_task())
            
    async def start_ranging(self):
        self.stop_ranging()
        self.writer.write(b'$PL,\r\n')
        # config_string = f'$PK,{self.ids["anchor_1"]},2,1,{self.ids["anchor_2"]},{self.ids["anchor_3"]},{self.ids["tag"]},\r\n'
        config_string = f'$PK,{self.ids["tag"]},0,3,{self.ids["anchor_1"]},{self.ids["anchor_2"]},{self.ids["anchor_3"]},\r\n'
        config_string = config_string.encode('utf-8')
        # print(config_string)
        self.writer.write(config_string)
        self.writer.write(b'$PS,\r\n')
        
    async def stop_ranging(self):
        self.writer.write(b'$PG,')
            
    async def _reader_task(self):
        msg=b''
        ii = 0
        while True:
            msg = await self.reader.readline()
            # print(msg)
            # msg = msg.strip().decode()
            # print(msg)
            try:
                cmd, par = msg.strip().split(b',',1)
            except:
                print(f'Failing to split message: {msg}')
                continue
            # cmd, par = cmd.decode(), par.decode()
            # print(cmd, par)
            if cmd == b'$PX':
                print(f'Ping received: {par}')
                
            elif cmd == b'$PD':
                ii += 1
                new_dist = np.nan*np.ones(3)
                try:
                    # print(f'Distances received: {par.split(b",")}')
                    tag_id, a1_dist, a2_dist, a3_dist, udata, _ = par.split(b',',5)
                    d1, d2, d3 = int(a1_dist, 16), int(a2_dist, 16), int(a3_dist, 16)
                    # print(f'Distances to {tag_id} are {d1}, {d2}, {d3} cm.')
                    if d1 == 0:
                        # print(f'Could not get distance from {tag_id}.')
                        pass
                    else:
                        new_dist[self.anchor_idx[tag_id]] = d1
                    self._distance_buffer.append(new_dist)
                except:
                    print(f'Could not decode distance message: {par}')
                    pass
                                            
                if ii % 100:
                    continue
                        
                self.distances = np.nanmean(np.stack(self._distance_buffer), axis=0)
                self.distances_sig = np.nanstd(np.stack(self._distance_buffer), axis=0)
                
                # print(self.distances)
                # print(self.distances_sig)
                self._distance_buffer = []
                # continue
                t0 = time.time()
                self.position = pos_solve(self.distances, self.anchor_pos, self.position)
                print(f'd = {self.distances.round(1)}; x = {self.position.round(1)}; t = {(time.time()-t0)*1000:.0f}')
                # if not (ii % 150):                    
                #     self.writer.write(b'$PI,B521,FF,20,30,\r\n')
                
            elif cmd == b'$PS':
                print(f'Ranging started.')
                
            elif cmd == b'$PG':
                print(f'Ranging stopped.')
                
            elif cmd == b'$PW':
                print(f'Configuration received: {par}')
                                
# async def call_func_with_msg(messages, func):
#     async for message in messages:
#         var = message.payload.decode("utf-8")
#         func(var)
        
# async def main():

#     client = mqtt.Client("localhost")
#     putzini = Putzini(client)

    
#     async with AsyncExitStack() as stack:
        
#         tasks = set()

#         await stack.enter_async_context(client)

#         manager = client.filtered_messages("putzini/move_r")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/move_r")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move_r)))
        
#         manager = client.filtered_messages("putzini/move_l")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/move_l")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move_l)))
        
#         manager = client.filtered_messages("putzini/move")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/move")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.move)))
        
#         manager = client.filtered_messages("putzini/turn")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/turn")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.turn)))
        
#         manager = client.filtered_messages("putzini/set_speed_r")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/set_speed_r")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.set_speed_r)))       
        
#         manager = client.filtered_messages("putzini/set_speed_l")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/set_speed_l")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.drive.set_speed_l)))
        
#         manager = client.filtered_messages("putzini/turn_absolute")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/turn_absolute")
#         tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.turn_absolute)))
        
#         manager = client.filtered_messages("putzini/move_absolute")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/move_absolute")
#         tasks.add(asyncio.ensure_future(await_func_with_msg(messages, putzini.move_absolute)))
        
#         manager = client.filtered_messages("putzini/head")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/head")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.lamp.set_head)))
        
#         manager = client.filtered_messages("putzini/neck")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/neck")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.move)))
                 
#         manager = client.filtered_messages("putzini/vacuum")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/vacuum")
#         tasks.add(asyncio.ensure_future(call_func_with_msg(messages, putzini.neck.set_vacuum)))

#         manager = client.filtered_messages("putzini/commands")
#         messages = await stack.enter_async_context(manager)
#         await client.subscribe("putzini/commands")
#         tasks.add(asyncio.ensure_future(parse_json_commands(messages, putzini)))

#         await asyncio.gather(*tasks)
        
async def main():

    # client = mqtt.Client("localhost")
    putzpos = PutziniPosition(mqtt_client=None)
    await putzpos.connect()
    await putzpos.start_ranging()
    while True:
        await asyncio.sleep(1)
    
if __name__ == '__main__':
    loop = asyncio.get_event_loop()      
    loop.run_until_complete(main())
    loop.close()