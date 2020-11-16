#!/usr/bin/env python
import pandas as pd
import json
from sys import argv, stderr
from urllib import request
import numpy as np
import re
from copy import deepcopy
import matplotlib.pyplot as plt

# TODO consider replacing the pandas dependency by csv (but I'm lazy)
if argv[1].endswith('xlsx'):
    df = pd.read_excel(argv[1]) 
elif argv[1].endswith('csv'):
    df = pd.read_csv(argv[1]) 
elif 'docs.google.com' in argv[1]:
    df = pd.read_csv(request.urlopen(argv[1] + '/export?gid=0&format=csv'))
else:
    raise ValueError('Input not recognized')

def expand_nested(dict_in):

    dict_out = {}
    
    for k, v in dict_in.items():    
        
        kseg = k.strip().split('.')        
        
        # iterate to the proper level
        d_cur = dict_out
        for _k in kseg[:-1]:
            if _k not in d_cur:
                d_cur[_k] = {}
            d_cur = d_cur[_k]
        
        kk = kseg[-1]
        
        if isinstance(v, str) and (v[0] == '{') and (v[-1] == '}'):
            # detect and evaluate dictionaries in cell
            
            try:
                d_cur[kk] = eval(v)
            except SyntaxError:
                print(v, f'in step {dict_in["name"]}, column {k}: is not a dict', file=stderr)
                d_cur[kk] = f'NO_DICT: ' + v
            except Exception as e:
                print(v, f'in step {dict_in["name"]}, column {k}: did not evaluate', file=stderr)
                d_cur[kk] = f'NOT_EVAL: ' + v
                      
        else:
            d_cur[kk] = v if pd.notna(v) else None
    
    return dict_out

# DataFrame of all steps
steps = df.dropna(axis=0, how='all').reset_index(drop=True)

# now iterate through steps manually as dict (tuple would not help here), to apply mods where required
steplist = []

#TODO extend this into a full state vector?
putz_pos = []

def parse_command(fun, npar, com):
    parsed = re.search(fun+'\(' + '\s*,\s*'.join(npar*['([\d +-]+)']) + '\)', com)
    return None if parsed is None else tuple(int(p) for p in parsed.groups())

for st in steps.to_dict(orient='records'):
    #TODO writing this as Spaghetti. Refactor one fine day.

    # PUTZINI ---
    pm = str(st['parameter.putzini.move'])

    if pm.startswith('moveToPos'):
        # just intercept to update current position and validate expression
        pp = parse_command('moveToPos', 3, pm)
        if pp is None:
            print(f'Non-compliant command in step {st["name"]}: {pm}', file=stderr)
        else:
            putz_pos.append(pp)
                            
        steplist.append(st)
        
    elif pm.startswith('moveRandomXY'):
        # unroll random motion into additional steps
        try:
            n_steps, xmin, xmax, ymin, ymax = parse_command('moveRandomXY', 5, pm)
            
        except Exception as err:
            print(f'Non-compliant command in step {st["name"]}: {pm}', file=stderr)
            print(str(err), file=stderr)
            steplist.append(st)
            continue
        
        for ii in range(n_steps):
            x = np.random.randint(xmin, xmax)
            y = np.random.randint(ymin, ymax)
            if ii == 0:
                rand_step = deepcopy(st) # don't mess with the iterator
            else:
                # for the extra steps we don't want the other parameters to repeat
                rand_step = {k: None for k in st.keys()}
            rand_step['name'] = st['name'] + f'_{ii:03d}'
            alpha = 0
            #TODO the angle here is nonsensical until there is a moveToPosXY command
            rand_step['parameter.putzini.move'] = f'moveToPos({x},{y},{alpha})'
            #TODO here the delta duration will still screw things up -> refactor into preprocessor?
            rand_step['duration'] = st['duration']/n_steps
            putz_pos.append((x, y, alpha))
            steplist.append(rand_step)
    
    elif pm.startswith('moveBy'):
        #TODO the angle issue (discuss with Sebastian)
        pp = parse_command('moveBy', 3, pm)
        if pp is None:
            print(f'Non-compliant command in step {st["name"]}: {pm}', file=stderr)
            steplist.append(st)
        else:
            new_pos = tuple(curr + off for curr, off in zip(putz_pos[-1], pp))
            st_rel = deepcopy(st)
            st_rel['parameter.putzini.move'] = f'moveToPos({new_pos[0]},{new_pos[1]},{new_pos[2]})'
            putz_pos.append(new_pos)
            steplist.append(st_rel)
                
    elif pm.startswith('moveToPosXY'):
        # move only XY, keeping the final alpha. If this is not possible directly, it could
        # simply compute the final alpha from the arctan of the previous points and convert
        # the call to moveToPos
        # OR: maybe better alternative... catch above if moveToPos is called with 2 (XY) or 1 (alpha) args only
        raise NotImplementedError('todo.')
    
    else:
        steplist.append(st)

final_steps = pd.DataFrame.from_records(steplist, columns=steps.columns) # giving cols to ensure consistency and order
final_steps.to_csv(argv[2].rsplit('.', 1)[0] + '_processed.csv', index=False)

expanded = {'steps': [expand_nested(st) for st in steplist]}

if len(argv) > 2:
    with open(argv[2], 'w') as fh:
        json.dump(expanded, fh, indent=2)    
else:
    print(json.dumps(expanded, indent=2, allow_nan=False))