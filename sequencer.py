#! /usr/bin/env python3
import json
import paho.mqtt.client as mqtt
import subprocess
from sys import argv
from time import sleep
import random

if len(argv) > 2:
    print('Updating story from:', argv[2])
    subprocess.run(['python3', 'tbl2json.py', argv[2], 'story.json'])

with open('story.json', 'r') as f:
    steps = json.load(f)['steps']

global mtrig, ptrig
mtrig, ptrig = False, False

def on_message(client, userdata, message):
    global mtrig, ptrig
    
    if 'putzini/state' in message.topic:
        ps = json.loads(message.payload.decode())
        if 'action' in ps and ps['action'] == 'Idle':
            if not ptrig:
                print('Putzini: IDLE received.')
                ptrig = True
    
    if 'music/state' in message.topic:
        ms = json.loads(message.payload.decode())
        if 'action' in ms and ms['action'] == 'Trigger':
            mtrig = True
            print('Music: Trigger received.')

client = mqtt.Client()
client.on_message = on_message
client.connect('172.31.1.150', 1883)
client.subscribe('putzini/state')
client.subscribe('music/state')
client.loop_start()

steps = steps[int(argv[1]):]

arka_state = {}
putzini_state = {}
try:

    for ii, s in enumerate(steps):
        
            
            print(f'------- STEP {ii}:', s['name'], '-------')
            if s['comment'] is not None:
                print(f'[{s["comment"]}]')
            
            # print(s)
            
            if s['parameter']['beamer'] is not None:
                for k in ['Regie', 'Arka', 'Putzini']:
                    if k in s['parameter']['beamer']:
                        print(f'{k} says ---')
                        print(s['parameter']['beamer'][k])
                        print('---')
                if 'Clear' in s['parameter']['beamer']:
                    print('--- Beamer cleared')
                    
            if s['parameter']['putzini'] is not None:
                client.publish('putzini/commands', json.dumps(s['parameter']['putzini']))
                ptrig = False
                putzini_state.update({k: v for k, v in s['parameter']['putzini'].items() if v is not None})
            
            if s['parameter']['arka'] is not None:
                arka_state.update({k: v for k, v in s['parameter']['arka'].items() if v is not None})
            
            print(f'Arka is doing: {arka_state}')
            print(f'Putzini is doing: {putzini_state}')
            
            if s['terminationCondition'] == 'Duration':
                t = float(s['duration'])
                t = t if s['deltaDuration'] is None else t + random.uniform(-s['deltaDuration'], s['deltaDuration'])
                print(f'Running step for {t:.1f} s...')
                sleep(t)
            elif s['terminationCondition'] == 'WaitForArka':
                input('Waiting for Arka. Press Enter to continue...')
            elif s['terminationCondition'] == 'WaitForMusic':
                print('Waiting for music...')
                while not mtrig:
                    sleep(0.1)
                mtrig = False
            elif s['terminationCondition'] == 'WaitForPutzini':
                print('Waiting for Putzini...')
                while not ptrig:
                    sleep(0.1)
                ptrig = False
            else:
                raise ValueError(f'Unknown termination condition: {s["terminationCondition"]}.')
        
except KeyboardInterrupt:
    print('Sending STOP to Putzini...')
    client.publish('putzini/commands', json.dumps({'move': 'stop()'}))
    sleep(1)
    print('Baba.')