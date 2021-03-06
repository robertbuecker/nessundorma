#!/usr/bin/env python3
"""
tbl2json
Story table to JSON converter for Robot Operas. Unwinds dots in column names to JSON hierarchies,
and evaluates JSON/Python-style dictionaries within cells.

Additionally mangles data for random/relative/... motion, specifically for Putzini.
Also it writes a file putzini.pdf illustrating Putzini's path.

Usage: tbl2json.py [IN] [OUT]
[IN] - csv or xlsx file, or link to Google Docs spreadsheet (without the final slash and anything after)
[OUT] - output JSON file
"""


import pandas as pd
import json
from sys import argv, stderr
from urllib import request
import numpy as np
import re
from copy import deepcopy
import matplotlib.pyplot as plt

# TODO consider replacing the pandas dependency by csv (but I'm too lazy)
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
                val = eval(v)
            except SyntaxError:
                print(v, f'in step {dict_in["name"]}, column {k}: is not a dict', file=stderr)
                val = f'NO_DICT: ' + v
            except Exception as e:
                print(v, f'in step {dict_in["name"]}, column {k}: did not evaluate', file=stderr)
                val = f'NOT_EVAL: ' + v
        
        elif isinstance(v, str) and v.isspace():
            v = None
        
        else:
            val = None if pd.isna(v) else v
            #TODO CATCH BLANKS

        if isinstance(val, float) and val.is_integer:
            # print('Converting float', k)
            val = int(round(val))
                
        d_cur[kk] = val
    
    return dict_out

def parse_command(fun, npar, com):
    # parses command com of form fun(par, par, par,.....) with npar parameters, all integer.
    # Returns a tuple.
    parsed = re.search(fun+'\(' + '\s*,\s*'.join(npar*['([\d +-]+)']) + '\)', com)
    return None if parsed is None else tuple(int(p) for p in parsed.groups())

def arka_validate(arka_par):
    # arka_par is a nested dict of all parameters below parameter.arka
    # return True if all is good.
    return True

# now iterate through steps manually as dict (tuple would not help here), to validate and mod where required
# DataFrame of all steps
steps = df.dropna(axis=0, how='all').reset_index(drop=True)
steplist = []
for st in steps.to_dict(orient='records'):
    #TODO writing this as Spaghetti. Refactor one fine day.

    skip_append = False # do not append step to final version at the end of iteration?
    stepname = st['name']
    log = lambda msg: print(f'In step {stepname}: {msg}', file=stderr)

    # PUTZINI MOVE VALIDATION --- (historical)
    pm = str(st['parameter.putzini.move'])
    pm_prev = steplist[-1]['parameter.putzini.move'] if steplist else None

    # ARKA VALIDATION ---
    # works a bit different from Putzini, as Arka has its paramters as hierarchy.
    # does not quite work yet, unfortunately, as it's tricky to reconstruct a full
    # state vector of Arka from the table.
    
    if False:
    # fill in a complete set of arka parameters...
        arka_par = {}
        for k in [k for k in steps.columns if k.startswith('parameter.arka')]:
            if (k in st) and pd.notna(st[k]) and (st[k] is not None):
                arka_par[k] = st[k]
            else:
                for st_prev in reversed(steplist):
                    if (k in st_prev) and pd.notna(st_prev[k]) and (st_prev[k] is not None):
                        arka_par = st[k]
                        break
                else:
                    arka_par[k] = None

        arka_par = expand_nested(arka_par)
        
        if not arka_validate(arka_par):
            log(f'Invalid arka parameters: {arka_par}')
    
    if not skip_append:
        steplist.append(st)

if False:
    fh, ax = plt.subplots(1,1)
    arka = plt.Circle((0, 0), 700, color='k', alpha=0.5)
    ax.add_artist(arka)
    ax.plot(putz_pos[:,0], putz_pos[:,1], ':o')
    plt.axis('equal')
    plt.savefig('putzini.pdf')

final_steps = pd.DataFrame.from_records(steplist, columns=steps.columns) # giving cols to ensure consistency and order
final_steps.to_csv(argv[2].rsplit('.', 1)[0] + '_processed.csv', index=False)

expanded = {'steps': [expand_nested(st) for st in steplist]}

if len(argv) > 2:
    with open(argv[2], 'w') as fh:
        json.dump(expanded, fh, indent=2)    
else:
    print(json.dumps(expanded, indent=2, allow_nan=False))