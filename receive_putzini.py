import paho.mqtt.client as mqtt
import numpy as np
import io
# from IPython.display import clear_output
from warnings import warn
import time
global all_T_mc, all_R_mc
all_T_mc, all_R_mc = None, None
euler_convention = 'ZYX'

from scipy.spatial.transform import Rotation as R

def recv_markers(client, userdata, msg):
    global all_T_mc, all_R_mc
    
    data = np.load(io.BytesIO(msg.payload))    
    all_T_mc, all_R_mc = data['tvecs'], data['rvecs']

client = mqtt.Client()
client.on_message = recv_markers
client.connect('172.31.1.150', 1883)
client.subscribe('putzini/markers')
client.loop_start()

# DEFINE TRANSFORMS
# _ab means: "a as seen from b", or equivalently "transforms from a system from b system"

all_T_mr = np.ones((50, 3))*np.nan
all_R_mr = np.ones((50, 3, 3))*np.nan

all_T_mr[0,...] = np.array([[0, 0, -235]])
all_T_mr[10,...] = np.array([[0, 100, -235]])
all_T_mr[11,...] = np.array([[0, 100, -235]])
all_T_mr[12,...] = np.array([[0, 100, -235]])

euler_R_mr = np.ones_like(all_T_mr) * np.nan
euler_R_mr[0,...] = [180,0,0]
euler_R_mr[10,...] = [0,0,0]
euler_R_mr[11,...] = [0,0,0]
euler_R_mr[12,...] = [0,0,0]

all_R_mr = np.stack([R.from_euler('ZXZ', ea, degrees=True).as_dcm() for ea in euler_R_mr])

T_cp = np.zeros(3) #position of cam from Putzini main axis
R_cp = R.from_euler('ZXZ', [0,0,0], degrees=True).as_dcm()

def trans(R=None, T=None, inv=False, X=None):
    R = np.eye(3) if R is None else R
    T = np.zeros(3) if T is None else T
    X = np.zeros(3) if X is None else X
    if inv:
        return np.dot(np.linalg.inv(R), X-T)
    else:
        return np.dot(R, X)+T

def trans_all(X_p=None):
    global all_T_mc, all_R_mc
    
    X_p = np.zeros(3) if X_p is None else X_p
    valid = np.any(np.isnan(all_T_mc), axis=1) | np.any(np.isnan(all_T_mr), axis=1)
    valid = np.nonzero(1-valid)[0]
    
    all_X_c = np.ones_like(all_T_mr) * np.nan
    all_X_m = np.ones_like(all_T_mr) * np.nan
    all_X_r = np.ones_like(all_T_mr) * np.nan
    all_R_pm = np.ones_like(all_R_mr) * np.nan
    all_R_pr = np.ones_like(all_R_mr) * np.nan
    
    for ii in valid:
        T_mc, R_mc = all_T_mc[ii,...], all_R_mc[ii,...]
        T_mr, R_mr = all_T_mr[ii,...], all_R_mr[ii,...]
        
        all_X_c[ii,...] = trans(R_cp, T_cp, True, X_p)
        all_X_m[ii,...] = trans(R_mc, T_mc, True, all_X_c[ii,...])
        all_X_r[ii,...] = trans(R_mr, T_mr, False, all_X_m[ii,...])
        all_R_pm[ii,...] = np.dot(np.linalg.inv(R_mc), np.linalg.inv(R_cp))
        all_R_pr[ii,...] = np.dot(R_mr, all_R_pm[ii,...])
        
    return valid, all_X_c, all_X_m, all_X_r, all_R_pm, all_R_pr

t0 = time.time()
while True:

    # clear_output(wait=True)
    
    time.sleep(0.5)
    
    pos = trans_all()

    print(f'Polled at dt={time.time() - t0:.2f}')
    
    if len(pos[0]) == 0:
        print('WARNING: Not seeing any marker!')
        continue

    valid_pos = [pos[0]] + [p[pos[0],...] for p in pos[1:]]
    pos_vs_room = valid_pos[3]/100
    pos_vs_markers = valid_pos[2]/100
    euler_vs_room = np.stack([R.from_dcm(r).as_euler(euler_convention, degrees=True) 
                              for r in valid_pos[5]])
    euler_vs_markers = np.stack([R.from_dcm(r).as_euler(euler_convention, degrees=True) 
                              for r in valid_pos[4]])

    print('Valid markers:', pos[0])
    print('Positions vs Room:\n', pos_vs_room)
    print('Euler angles vs Room:\n', euler_vs_room)

    print('Positions vs Markers:\n', pos_vs_markers)
    print('Euler angles vs Markers:\n', euler_vs_room)

#     time.sleep(0.5)