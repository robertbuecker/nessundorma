#!/usr/bin/env python3

import subprocess
import json
from sys import argv
from hashlib import md5
import os

fn = 'story.json' if len(argv) < 2 else argv[1]
os.makedirs('speech', exist_ok=True)

with open(fn) as fh:
    story = json.load(fh)
    
lines = []
for step in story['steps']:
    try:
        ln = step['parameter']['beamer']['Regie']
        lines.append(ln)
    except (KeyError, TypeError):
        continue
    
for ln in lines:
    print(ln)
    fn = os.path.join('speech', md5(ln.encode()).hexdigest() + '.wav')
    if not os.path.exists(fn):   
        print(fn, 'not found, synthesizing...')
        subprocess.run(['say', '-o', fn, '--data-format=LEI16@22050', ln])
        print('done!')
    else :
        print(fn, 'already exists!')
    print('----')