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

async def play_labeled_file(wave_file, label_file=None):

    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    opera = sa.WaveObject.from_wave_file(wave_file)   
    
    if label_file is not None:
        timing = TimedMessageDispatcher(client)
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
        timing.label_list = lbls
        await timing.start()
            
    playobj = opera.play()
    while playobj.is_playing():
        await asyncio.sleep(0.1)
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(play_labeled_file(argv[1] if len(argv) >= 2 else 'opera.wav', argv[1] if len(argv) >= 3 else 'opera.txt'))
    loop.close()