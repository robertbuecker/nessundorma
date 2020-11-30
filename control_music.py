#!/usr/bin/env python
# coding: utf-8

import paho.mqtt.client as mqtt
import simpleaudio as sa
import time
import pandas as pd
import json
import numpy as np
import sys
import pipeclient
from warnings import warn


use_audacity = True
use_aud_list = True

if use_audacity:
    aud = pipeclient.PipeClient()
else:
    opera = sa.WaveObject.from_wave_file('opera.wav')    

# Define set of functions for start and stop
if use_audacity:
    def play(start=0):
        aud.write('Stop')
        if start == 0:
            aud.write('CursProjectStart')
        else:
            startstr = f'{start}'.replace('.', ',') # REMOVE FOR ENGLISH AUDACITY
            aud.write(f'SelectTime: Start={startstr} End={startstr} RelativeTo=ProjectStart')
        # time.sleep(0.1)
        aud.write('Play')
        return time.time() - start
    def stop():
        # pass
        aud.write('Stop')
else:
    def play():
        player = opera.play()
        return time.time()

if use_audacity and use_aud_list:
    aud.write('GetInfo: Type=Labels')
    time.sleep(0.2)
    lbl0 = pd.DataFrame(json.loads(aud.read().split('BatchCommand')[0])[0][1])
    lbl0.columns = ['start', 'end', 'label']
else:
    lbl0 = pd.read_csv('opera.txt', sep='\t', header=None)
    lbl0.columns = ['start', 'end', 'label']
    
# Table sanitizer
lbl0 = lbl0.drop(columns='label').join(
    lbl0.label.str.split(
        ',', expand=True).rename(
        columns={0: 'comment', 1: 'speed', 2: 'trigger'}))
lbl0.trigger = lbl0.trigger.str.lower().str.strip() == 't'
lbl0.speed = pd.to_numeric(lbl0.speed).fillna(-1).astype(int)

if len(sys.argv) > 1:
    start_label = sys.argv[1]
    start_time = lbl0.query(f'comment == "{start_label}"').start.values
    if len(start_time) != 1:
        raise ValueError('Non-existing or non-unique starting position:', start_label)
    start_time = start_time[0]
    print('Starting from time:', start_time)
else:
    start_label = None
    start_time = 0

lbl0 = lbl0.loc[lbl0.start > start_time,:]

print('Loaded score table. Starts with:\n', lbl0.head(5))

global waiting_for
waiting_for = False
def on_message(client, userdata, message):
    global waiting_for
    if 'WaitForMusic' in message.payload.decode():
        msg = json.loads(message.payload.decode())
        step_name = msg['WaitForMusic']
        waiting_for = step_name if step_name else 'unlabeled'
        # print('Waiting in:', msg['WaitForMusic'])

client = mqtt.Client()
client.on_message = on_message
client.connect('172.31.1.150', 1883)
client.subscribe('music/commands')
client.loop_start()

print(f'Starting Opera. Hit Ctrl-C to stop it.')
t0 = play(start_time)
last_msg = client.publish('music/state', json.dumps({'action': 'Trigger'}))
lbl = lbl0.copy()
qt = [] # trigger queue

try:   
    while len(lbl) > 0: 
        
        ela = time.time() - t0
        passed = lbl[lbl.start <= ela]
        lbl.drop(passed.index, inplace=True)
               
        # Any missing Triggers?
        if (len(qt) > 0) and waiting_for:
            comm = qt.pop(0)
            print(f'{ela:.1f} WARNING: Skipping step {wait_lbl} due to already received trigger {comm}')
            last_msg = client.publish('music/state', json.dumps({'action': 'Trigger'}))
            waiting_for = ''
            time.sleep(0.05)
      
        for event in passed.itertuples():
            msg = {}
            if event.trigger:
                if waiting_for:
                    msg['action'] = 'Trigger'
                    print(f'Sending trigger {event.comment} to finish step {waiting_for}.')
                    waiting_for = ''
                    
                else:
                    # Trigger while not waiting for one...
                    qt.append(event.comment)
                    print(f'{ela:.1f} WARNING: Trigger {event.comment} requested by music while not waiting for any. Check timing!')
            
            if event.speed > -1:
                print(f'{ela:.1f} Changing speed to {event.speed} as requested by {event.comment}.')                
                msg['speed'] = event.speed
            
            if len(msg) > 0:
                # print('{ela:.1f} Sending message:', msg)
                last_msg = client.publish('music/state', json.dumps(msg))
                
            else:
                print(f'{ela:.1f} Passing Label without action: {event.comment}.')
                
                
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print('Opera finished.')
    
except Exception as err:
    raise err

finally:
    stop()
    client.loop_stop()           
    client.unsubscribe('music/commands')
    client.disconnect()
    