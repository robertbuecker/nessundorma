#!/usr/bin/env python
# coding: utf-8

import asyncio_mqtt as mqtt
import simpleaudio as sa
import time
import json
import numpy as np
import sys
from warnings import warn
import asyncio
from timed_message_dispatcher import TimedMessageDispatcher
from putzini_track import PutziniTrack
import csv
import asyncio_mqtt as mqtt
from putzini_config import PutziniConfig
import logging
import os

logger = logging.getLogger(__name__)

class MusicPlayer:
    
    def __init__(self, mqtt_client: mqtt.Client):
        self.mqtt_client = mqtt_client
        self.tracks = []
        self.logger = logger
        
    def start(self):
        asyncio.ensure_future(self.listener())

    async def listener(self):
        self.logger.info('Started listening.')
        client = self.mqtt_client
        async with client.filtered_messages('player/commands') as messages:
            await client.subscribe('player/commands')
            async for message in messages:
                self.logger.debug('Received message (%s) %s: %s', message.timestamp, message.topic, message.payload)
                
                if 'stop' in message.payload.decode():
                    try:
                        msg = json.loads(message.payload.decode())['stop']
                        if msg is not None:
                            if (not isinstance(msg, dict)) or ('file' not in msg):
                                self.logger.info('Stopping all playing files.')
                                self.stop_all()
                                
                            else:
                                self.stop(os.path.join(msg["folder"] if 'folder' in msg else '.', msg["file"]))
                            
                    except Exception as err:
                        self.logger.exception('Failed to interpret stop payload: %s', message.payload)
                                                
                
                if 'play' in message.payload.decode():
                    try:
                        msg = json.loads(message.payload.decode())['play']
                        if msg is not None:
                            file_name = msg['file'] if 'file' in msg else None
                            folder_name = msg['folder'] if 'folder' in msg else '.'
                            loop = bool(int(msg['loop'])) if 'loop' in msg else False
                            labels = msg['labels'] if 'labels' in msg else None
                            vol = float(msg['vol']) if 'vol' in msg else 100.
                            start = float(msg['start']) if 'start' in msg else 0.
                            new_track = PutziniTrack(wave_file=None if file_name is None else os.path.join(folder_name, file_name), 
                                                    label_file=None if labels is None else os.path.join(folder_name, labels), 
                                                    start_time = start, volume=vol,
                                                    mqtt_client=self.mqtt_client)
                            self.tracks.append(new_track)
                            asyncio.ensure_future(new_track.play(loop=loop))
                        
                    except:
                        self.logger.exception('Failed to initialize playing track %s', file_name)
                        
                await asyncio.sleep(0.2)
                play_waves = [t.wave_file for t in self.playing_tracks]
                asyncio.ensure_future(self.mqtt_client.publish('player/state', 
                                                               json.dumps({'playing': play_waves})))
                        
    @property
    def playing_tracks(self):
        return [t for t in self.tracks if t.is_playing()]
    
    @property
    def N_playing(self):
        return len(self.playing_tracks)
    
    def stop_all(self):
        for t in self.playing_tracks:
            t.stop()
            
    def stop(self, filename):
        self.logger.info('Trying to stop file %s', filename)
        for t in self.playing_tracks:
            if t.wave_file == filename:
                t.stop()
                break
                
        else:
            self.logger.warn('Cannot stop track with filename %s: not found or playing.', filename)


## FROM HERE: ACTUAL SINGING
async def main():

    config = PutziniConfig()
    async with mqtt.Client(config.mqtt_broker) as client:
        player = MusicPlayer(client)
        player.start()
        while True:
            await asyncio.sleep(1)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()