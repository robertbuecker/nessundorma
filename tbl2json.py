#!/usr/bin/env python3
"""
tbl2json
Story table to JSON converter for Robot Operas. Unwinds dots in column names to JSON hierarchies,
and evaluates JSON/Python-style dictionaries within cells.

Additionally mangles data for random/relative/... motion, specifically for Putzini.
Also it writes a file putzini.pdf illustrating Putzini's path.

Usage: tbl2json.py [IN1] [IN2] [IN3] ... [OUT]
[IN] - csv or xlsx files, or link to Google Docs spreadsheet (without the final slash and anything after),
        steps from those will be concatenated.
[OUT] - output JSON file
"""


import pandas as pd
import json
from sys import argv, stderr
from urllib import request
import numpy as np
import re
from copy import deepcopy
from hashlib import md5

dfs = []
fn_out = None
for ii, tbl_name in enumerate(argv[1:-1]):
    
    # TODO consider replacing the pandas dependency by csv (but I'm too lazy)
    if tbl_name.endswith('xlsx'):
        df = pd.read_excel(tbl_name) 
    elif tbl_name.endswith('csv'):
        df = pd.read_csv(tbl_name) 
    elif 'docs.google.com' in tbl_name:
        url = tbl_name + '/export?gid=0&format=csv'
        print('Getting table from:', url)
        df = pd.read_csv(request.urlopen(url))
    else:
        raise ValueError(f'Input not recognized: {tbl_name}')
    
    dfs.append(df)
    
df = pd.concat(dfs, axis=0, ignore_index=True)

if argv[-1].endswith('json'):
        # it's the output file. Yep, that is dirty.
    fn_out = argv[-1]
else:
    raise ValueError('Output filename must end with .json')

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
col = [c for c in steps.columns if c.strip() == 'parameter.beamer'][0]

print('Have', len(steps), 'steps.', file=stderr)

for st in steps.to_dict(orient='records'):

    skip_append = False # do not append step to final version at the end of iteration?
    stepname = st['name']
    log = lambda msg: print(f'In step {stepname}: {msg}', file=stderr)

    try:
        ln = json.loads(st[col])['Regie']
    except (KeyError, TypeError):
        ln = None
        pass
    
    if ln is not None:
        fn = 'speech/' + md5(ln.encode()).hexdigest() + '.wav'
        print(f'Speech line: "{ln}"., Inferred filename: {fn}.', file=stderr)
        st['parameter.player.play'] = json.dumps({'file': fn})

    if not skip_append:
        steplist.append(st)

final_steps = pd.DataFrame.from_records(steplist, columns=steps.columns) # giving cols to ensure consistency and order
fn_csv = fn_out.rsplit('.', 1)[0] + '_processed.csv'
final_steps.to_csv(fn_csv, index=False)
print(f'Wrote final table to {fn_csv}', file=stderr)

expanded = {'steps': [expand_nested(st) for st in steplist]}

if fn_out is not None:
    with open(fn_out, 'w') as fh:
        json.dump(expanded, fh, indent=2)
    print(f'Wrote {len(expanded["steps"])} steps to {fn_out}')
    
else:
    print(json.dumps(expanded, indent=2, allow_nan=False))