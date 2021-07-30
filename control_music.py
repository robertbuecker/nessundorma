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
import csv
import asyncio_mqtt as mqtt
from putzini_config import PutziniConfig
import logging

logger = logging.getLogger(__name__)

## FROM HERE: ACTUAL SINGING
async def main():

    config = PutziniConfig()
    client = mqtt.Client(config.mqtt_broker)
    timing = TimedMessageDispatcher(client)
    opera = sa.WaveObject.from_wave_file(sys.argv[1] if len(sys.argv) >= 2 else 'opera.wav')   
    with open(sys.argv[2] if len(sys.argv) >= 3 else 'opera.txt', newline='') as fh:
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
        await asyncio.sleep(1)

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    loop = asyncio.get_event_loop()      
    # loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()