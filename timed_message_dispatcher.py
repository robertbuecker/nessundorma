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
from typing import Optional

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
        self.label_list = []
        self.t0 = 0.
        self.ela = 0.
        self._running = False
        self._ea = None
        self._st = None
        self.N_labels = 0
        self.N_triggers = 0
        
    def set_label_list(self, label_list: Union[list, tuple]):

        self.N_labels = len(label_list)
        self.N_triggers = len([l for l in label_list if l["trigger"]])        
        self.label_list = label_list
        self.logger.info('Have label list with %s labels, %s of which are triggers.', self.N_labels, self.N_triggers)
        
    def add_arm_message(self, topic, payload: Optional[str] = None, json_key: Optional[str] = None):
        self._arm_messages.append({topic: (payload, json_key)})
        
    def add_trigger_message(self, topic, payload: Optional[str] = None, json_keys: Optional[Union[list, tuple]] = (), send: bool = True):
        self._trigger_messages.append({topic: (payload, json_keys, send)})
        
    def start(self):
        self.t0 = time()
        self._ea = asyncio.ensure_future(self.enqueue_arms())
        self._st = asyncio.ensure_future(self.send_triggers())
        self.logger.info('Timed message dispatcher started.')
        self._running = True
        
    def stop(self):
        if self._ea is not None:
            self._ea.cancel()
        if self._st is not None:
            self._st.cancel()
        self.logger.info('Timed message dispatcher stopped.')
            
    def is_running(self):
        if (self._ea is None) or (self._st is None):
            return False
        return not (self._ea.done() and self._st.done())
        
    async def enqueue_arms(self):
        client = self.mqtt_client
        async with client.filtered_messages('music/commands') as messages:
            await client.subscribe('music/commands')
            async for message in messages:
                if 'WaitForMusic' in message.payload.decode():
                    try:
                        msg = json.loads(message.payload.decode())
                        step_name = msg['WaitForMusic']
                        
                    except Exception as err:
                        self.logger.error('(%.1f) Failed to interpret arming payload: %s', self.ela, message.payload)
                        step_name = f'MALFORMED MESSAGE'
                        
                    self.arm_q.append(step_name if step_name else 'unlabeled')
                    self.logger.info('(%.1f) Received WaitForMusic: %s, Arm Q has %s entries now.', self.ela, step_name, len(self.arm_q))
                
                if 'play' in message.payload.decode():
                    self.logger.warning('(%.1f) Received play message: %s', self.ela, message.payload)
                    
    async def send_triggers(self):
                
        self.logger.debug('Starting trigger loop with %s labels.')
                
        while True:
            
            while self.trigger_q and self.arm_q:
                com_trig = self.trigger_q.popleft()
                com_arm = self.arm_q.popleft()
                self.logger.warning('(%.1f) Immediately answering %s due to deferred trigger %s', self.ela, com_arm, com_trig)
                asyncio.ensure_future(self.mqtt_client.publish('music/state', json.dumps({'action': 'Trigger'})))
                await asyncio.sleep(0.05)
            
            # this is SO tedious without pandas...
            # self.logger.debug('(%.1f) Next label is %s.', self.ela, self.label_list[0])
            
            passed = [lbl for lbl in self.label_list if lbl['time'] <= self.ela]
            self.label_list = [lbl for lbl in self.label_list if lbl['time'] > self.ela]
            
            self.logger.debug('%s labels passed since last iteration; %s remaining', len(passed), len(self.label_list))
            
            self.ela = time() - self.t0
            
            for event in passed:
                
                self.logger.debug('Processing event %s', event)

                msg = {}
                if event["trigger"] and self.arm_q:
                    # we have an awaited trigger!
                    waiting_for = self.arm_q.popleft()
                    if event["trigger"]:
                        msg['action'] = 'Trigger'
                        self.logger.info('(%.1f) Sending trigger %s to finish step %s', self.ela, event["comment"], waiting_for)
                    
                elif event["trigger"]:
                    # we don't have an awaited trigger?!
                    self.trigger_q.append(event["comment"])
                    self.logger.warning('(%.1f) Trigger %s requested while not awaiting one. Trigger Q now has %s entries.', 
                                        self.ela, event["comment"], len(self.trigger_q))
                    
                if ("speed" in event) and (event["speed"] is not None) and (event["speed"] > -1):
                    msg["speed"] = event["speed"]
                    self.logger.info('(%.1f) Changing speed to %s as requested by %s', self.ela, event["speed"], event["comment"])
                    
                if len(msg) > 0:
                    self.logger.debug('(%.1f) Sending message: %s', self.ela, msg)
                    asyncio.ensure_future(self.mqtt_client.publish('music/state', json.dumps(msg)))
                    
                else:
                    self.logger.info('(%.1f) Passed label %s without sending anything.', self.ela, event["comment"])

            await asyncio.sleep(0.05)
            
            if not self.label_list:
               self.logger.debug('(%.1f) Passed last label in list', self.ela)

async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    async with mqtt.Client(config.mqtt_broker) as client:
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
        timing.start()
        while True:
            await asyncio.sleep(1)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()