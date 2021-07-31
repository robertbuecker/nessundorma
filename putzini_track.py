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

# logger = logging.getLogger(__name__)

class PutziniTrack:
    
    def __init__(self, wave_file=None, label_file=None, mqtt_client=None):

        self.name = ','.join([fn for fn in [wave_file, label_file] if fn is not None])

        self.logger = logging.getLogger(f'putzini_track:{self.name}')
        
        if (wave_file is None) and (label_file is None):
            raise ValueError('You must either specify a wave file or a label file!')
        
        self.wave_file = wave_file
        self.label_file = label_file
        self.mqtt_client = mqtt_client
        
        
        self.wave = None
        self.timing = None
        self._loop = False
        self.playback = sa.PlayObject(0)
        
        
        if wave_file is not None:
            self.wave = sa.WaveObject.from_wave_file(wave_file)
        
        if (label_file is not None) and (self.mqtt_client is not None):
            self.timing = TimedMessageDispatcher(self.mqtt_client)
            with open(self.label_file, newline='') as fh:
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
            
            self.logger.info('Have label list with %s entries', len(lbls))
            self.timing.label_list = lbls
    
    async def play(self, loop=False):
        # this is quasi-blocking!
        self._loop = loop
        while True:
            if self.timing is not None:
                self.timing.start()
                self.logger.info('Timing labels %s started', self.timing)
            if self.wave is not None:
                self.playback = self.wave.play()
                self.logger.info('Playback of %s started', self.wave_file)
            while self.is_playing():
                await asyncio.sleep(0.1)
            if not self._loop:
                break
            
    def stop(self):
        self._loop = False
        self.playback.stop()
        if self.timing is not None:
            self.timing.stop()
        self.logger.info('Playback of %s stopped', self.wave_file)
            
    def is_playing(self):
        return (self.playback.is_playing()
                or (self.timing.is_running() if self.timing is not None else False))
    
async def main():
    import asyncio_mqtt as mqtt
    from putzini_config import PutziniConfig
    config = PutziniConfig()
    async with mqtt.Client(config.mqtt_broker) as client:
        opera = PutziniTrack(argv[1] if len(argv) >= 2 else 'opera.wav', argv[1] if len(argv) >= 3 else 'opera.txt', client)
        if False:
            await opera.play()
        else:
            asyncio.ensure_future(opera.play())
            await asyncio.sleep(10)
            opera.stop()

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()