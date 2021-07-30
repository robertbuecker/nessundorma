#!/usr/bin/env python3

import asyncio_mqtt as mqtt
import asyncio
from time import time
import json
import asyncio
from typing import Optional, Union
from collections import deque, defaultdict
from time import time
from sys import argv
import csv

import logging

logger = logging.getLogger(__name__)

class TimedMessageDispatcher:

    def __init__(self, mqtt_client: mqtt.Client):
        self.logger = logger
        self.arm_json = True
        self._arm_messages = []
        self._trigger_messages = []
        self.mqtt_client = mqtt_client
        self.arm_q = deque([])
        self.trigger_q = deque([])
        self.label_list = defaultdict
        self.t0 = 0.
        
    def add_arm_message(self, topic, payload: Optional[str] = None, json_key: Optional[str] = None):
        self._arm_messages.append({topic: (payload, json_key)})
        
    def add_trigger_message(self, topic, payload: Optional[str] = None, json_keys: Optional[Union[list, tuple]] = (), send: bool = True):
        self._trigger_messages.append({topic: (payload, json_keys, send)})
        
    async def start(self):
        self.t0 = time()
        asyncio.ensure_future(self.enqueue_arms())
        asyncio.ensure_future(self.send_triggers())
        self.logger.info('Trigger/Arm loops started.')
        
    async def enqueue_arms(self):
        async with self.mqtt_client as client:
            async with client.filtered_messages('music/commands') as messages:
                await client.subscribe('music/commands')
                async for message in messages:
                    if 'WaitForMusic' in message.payload.decode():
                        try:
                            msg = json.loads(message.payload.decode())
                            step_name = msg['WaitForMusic']
                            
                        except Exception as err:
                            self.logger.error('Failed to interpret arming payload: %s', message.payload)
                            step_name = f'MALFORMED MESSAGE'
                            
                        self.arm_q.append(step_name if step_name else 'unlabeled')
                        waiting_for = step_name if step_name else 'unlabeled'
                        self.logger.info('Waiting in:', waiting_for)
                    
    async def send_triggers(self):
        
        self.logger.debug('Trigger loop started.')
        
        while len(self.label_list) > 0:
            
            while self.trigger_q and self.arm_q:
                com_trig = self.trigger_q.popleft()
                com_arm = self.arm_q.popleft()
                self.logger.warning('Immediately answering %s due to waiting trigger %s', ela, com_arm, com_trig)
                asyncio.ensure_future(self.mqtt_client.publish('music/state', json.dumps({'action': 'Trigger'})))
                await asyncio.sleep(0.05)
            
            # this is SO tedious without pandas...
            ela = time() - self.t0
            self.logger.debug('(%.1f) Next label is %s.', ela, self.label_list[0])
            
            passed = [lbl for lbl in self.label_list if lbl['time'] <= ela]
            self.label_list = [lbl for lbl in self.label_list if lbl['time'] > ela]
            
            self.logger.debug('%s labels passed since last iteration; %s remaining', len(passed), len(self.label_list))
            
            for event in passed:
                
                self.logger.debug('Processing event %s', event)

                msg = {}
                if event["trigger"] and self.arm_q:
                    # we have an awaited trigger!
                    waiting_for = self.arm_q.popleft()
                    if event["trigger"]:
                        msg['action'] = 'Trigger'
                        self.logger.info('(%.1f) Sending trigger %s to finish step %s', ela, event["comment"], waiting_for)
                    
                elif event["trigger"]:
                    # we don't have an awaited trigger?!
                    self.trigger_q.append(event["comment"])
                    self.logger.warning('(%.1f) Trigger %s requested while not awaiting one. Trigger Q now has %s entries', 
                                        ela, event["comment"], len(self.trigger_q))
                    
                if ("speed" in event) and (event["speed"] is not None) and (event["speed"] > -1):
                    msg["speed"] = event["speed"]
                    self.logger.info('(%.1f) Changing speed to %s as requested by %s', ela, event["speed"], event["comment"])
                    
                if len(msg) > 0:
                    self.logger.debug('(%.1f) Sending message: %s', ela, msg)
                    asyncio.ensure_future(self.mqtt_client.publish('music/state', json.dumps(msg)))
                    
                else:
                    self.logger.warning('(%.1f) Passed label %s without action %s', ela, event["comment"])
                    
                if not event["trigger"]:
                    self.logger.info('(%.1f) Passed event %s without trigger.', ela, event["comment"])
                    
            await asyncio.sleep(1)

async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    timing = TimedMessageDispatcher(client)
    
    with open(argv[1], newline='') as fh:
        reader = csv.DictReader(fh, delimiter='\t', fieldnames=['start', 'end', 'text'])
        lbls = []
        for row in reader:
            stp = {}
            stp['time'] = float(row['start'])
            txt = row['text'].split(',')
            stp['comment'] = txt[0]
            stp['speed'] = int(txt[1]) if txt[1] else None
            stp['trigger'] = txt[2].strip() == 'T'
            lbls.append(stp)
            
    logger.info('Have label list with %s entries', len(lbls))
    timing.label_list = lbls
    await timing.start()
    while True:
        await asyncio.sleep(1)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()