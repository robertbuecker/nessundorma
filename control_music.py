#!/usr/bin/env python3

import asyncio_mqtt as mqtt
import simpleaudio as sa
import time
import json
import numpy as np
from sys import argv
from warnings import warn
import asyncio
from timed_message_dispatcher import TimedMessageDispatcher
import csv
import asyncio_mqtt as mqtt
from putzini_config import PutziniConfig
import logging

logger = logging.getLogger(__name__)

class PutziniTrack:
    
    def __init__(self, wave_file, label_file=None, mqtt_client=None):
        
        self.wave_file = wave_file
        self.wave = sa.WaveObject.from_wave_file(wave_file)
        self.playback = sa.PlayObject(0)
        self.timing = None
        self.logger = logger
        self.mqtt_client = mqtt_client
        
        if (label_file is not None) and (self.mqtt_client is not None):
            self.timing = TimedMessageDispatcher(self.mqtt_client)
            with open(label_file, newline='') as fh:
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
            self.timing.label_list = lbls
    
    async def play(self, loop=False):
        # this is quasi-blocking!
        while True:
            if self.timing is not None:
                self.timing.start()
            self.playback = self.wave.play()
            self.logger.info('Playback of %s started', self.wave_file)
            while self.is_playing():
                await asyncio.sleep(0.1)
            if not loop:
                break
            
            
    async def stop(self):
        self.playback.stop()
        if self.timing is not None:
            self.timing.stop()
        self.logger.info('Playback of %s stopped', self.wave_file)
            
    def is_playing(self):
        return self.playback.is_playing()
    
async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    opera = PutziniTrack(argv[1] if len(argv) >= 2 else 'opera.wav', argv[1] if len(argv) >= 3 else 'opera.txt', client)
    asyncio.ensure_future(opera.play())
    await asyncio.sleep(10)
    await opera.stop()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()