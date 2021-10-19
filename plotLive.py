#!/usr/bin/env python3

# -*- coding: utf-8 -*-
"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

# import cv2
from scipy.spatial.transform import Rotation


from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import numpy as np
import pyqtgraph as pg
import io
import json
import yaml
from ast import literal_eval
from imageio import imread

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

import paho.mqtt.client as mqtt
from putzini_config import PutziniConfig

class PutziniError(Exception):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

class MqttClient(QtCore.QObject):
    Disconnected = 0
    Connecting = 1
    Connected = 2

    MQTT_3_1 = mqtt.MQTTv31
    MQTT_3_1_1 = mqtt.MQTTv311

    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()

    stateChanged = QtCore.pyqtSignal(int)
    hostnameChanged = QtCore.pyqtSignal(str)
    portChanged = QtCore.pyqtSignal(int)
    keepAliveChanged = QtCore.pyqtSignal(int)
    cleanSessionChanged = QtCore.pyqtSignal(bool)
    protocolVersionChanged = QtCore.pyqtSignal(int)

    messageSignal = QtCore.pyqtSignal(str)
    voltageSignal = QtCore.pyqtSignal(float)
    distancesSignal = QtCore.pyqtSignal(dict)
    calibratedSignal = QtCore.pyqtSignal(tuple)
    stateSignal = QtCore.pyqtSignal(dict)
    command_stateSignal = QtCore.pyqtSignal(dict)
    logsSignal = QtCore.pyqtSignal(str)

    errorSignal = QtCore.pyqtSignal(PutziniError)

    def __init__(self, parent=None):
        super(MqttClient, self).__init__(parent)

        self.m_hostname = ""
        self.m_port = 1883
        self.m_keepAlive = 60
        self.m_cleanSession = True
        self.m_protocolVersion = MqttClient.MQTT_3_1

        self.m_state = MqttClient.Disconnected

        self.m_client =  mqtt.Client(clean_session=self.m_cleanSession,
            protocol=self.protocolVersion)

        self.m_client.on_connect = self.on_connect
        self.m_client.on_message = self.on_message
        self.m_client.on_disconnect = self.on_disconnect

    @QtCore.pyqtProperty(int, notify=stateChanged)
    def state(self):
        return self.m_state

    @state.setter
    def state(self, state):
        if self.m_state == state: return
        self.m_state = state
        self.stateChanged.emit(state) 

    @QtCore.pyqtProperty(str, notify=hostnameChanged)
    def hostname(self):
        return self.m_hostname

    @hostname.setter
    def hostname(self, hostname):
        if self.m_hostname == hostname: return
        self.m_hostname = hostname
        self.hostnameChanged.emit(hostname)

    @QtCore.pyqtProperty(int, notify=portChanged)
    def port(self):
        return self.m_port

    @port.setter
    def port(self, port):
        if self.m_port == port: return
        self.m_port = port
        self.portChanged.emit(port)

    @QtCore.pyqtProperty(int, notify=keepAliveChanged)
    def keepAlive(self):
        return self.m_keepAlive

    @keepAlive.setter
    def keepAlive(self, keepAlive):
        if self.m_keepAlive == keepAlive: return
        self.m_keepAlive = keepAlive
        self.keepAliveChanged.emit(keepAlive)

    @QtCore.pyqtProperty(bool, notify=cleanSessionChanged)
    def cleanSession(self):
        return self.m_cleanSession

    @cleanSession.setter
    def cleanSession(self, cleanSession):
        if self.m_cleanSession == cleanSession: return
        self.m_cleanSession = cleanSession
        self.cleanSessionChanged.emit(cleanSession)

    @QtCore.pyqtProperty(int, notify=protocolVersionChanged)
    def protocolVersion(self):
        return self.m_protocolVersion

    @protocolVersion.setter
    def protocolVersion(self, protocolVersion):
        if self.m_protocolVersion == protocolVersion: return
        if protocolVersion in (MqttClient.MQTT_3_1, MQTT_3_1_1):
            self.m_protocolVersion = protocolVersion
            self.protocolVersionChanged.emit(protocolVersion)

    #################################################################
    @QtCore.pyqtSlot()
    def connectToHost(self):
        if self.m_hostname:
            self.m_client.connect(self.m_hostname, 
                port=self.port, 
                keepalive=self.keepAlive)

            self.state = MqttClient.Connecting
            self.m_client.loop_start()

    @QtCore.pyqtSlot()
    def disconnectFromHost(self):
        self.m_client.disconnect()

    def subscribe(self, path):
        if self.state == MqttClient.Connected:
            self.m_client.subscribe(path)

    #################################################################
    # callbacks
    def on_message(self, mqttc, obj, msg):
        if msg.topic == 'putzini/position':
            mstr = msg.payload.decode("utf-8")
            # print("on_message", mstr, obj, mqttc)
            self.messageSignal.emit(mstr)
        elif msg.topic == 'putzini/error':
            mstr = msg.payload.decode('utf-8')
            self.errorSignal.emit(PutziniError(mstr))
        elif msg.topic == 'putzini/distances':
            distance_data = json.loads(msg.payload.decode('utf-8'))
            self.distancesSignal.emit(distance_data)
        elif msg.topic == 'putzini/calibrated':
            calibrated = tuple(literal_eval(msg.payload.decode('utf-8')))
            self.calibratedSignal.emit(calibrated)
        elif msg.topic == 'putzini/state':
            state = json.loads(msg.payload.decode('utf-8'))
            self.stateSignal.emit(state)
        elif msg.topic == 'putzini/command_state':
            command_state = json.loads(msg.payload.decode('utf-8'))
            self.command_stateSignal.emit(command_state)
        elif msg.topic == 'putzini/logs':
            log_message = msg.payload.decode('utf-8')
            self.logsSignal.emit(log_message)                                    
        else:
            # print(msg.topic, msg.payload)
            mstr = float(msg.payload.decode("utf-8"))
            self.voltageSignal.emit(mstr)

    def on_connect(self, *args):
        # print("on_connect", args)
        self.state = MqttClient.Connected
        self.connected.emit()

    def on_disconnect(self, *args):
        # print("on_disconnect", args)
        self.state = MqttClient.Disconnected
        self.disconnected.emit()

#QtGui.QApplication.setGraphicsSystem('raster')
#app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

_mos = np.asarray([
    [0.5, 0.25],
    [0.5, -0.25],
    [-0.5, 0.25],
    [-0.5, -0.25],
    [0.5, 0.25]
])
_mos = np.asarray([
    [-0.5, 0],
    [0.5, 0.5],
    [0, 0],
    [0.5, -0.5],
    [-0.5, 0]
])
my_symbol = pg.arrayToQPath(_mos[:, 0], _mos[:, 1], connect='all')

tilt_fig = False
jitter_limit = 0.01 # in m

class MainWindow(QtGui.QWidget):
    def __init__(self):
        # super().__init__(show=True, title="Putzini Tracker")
        super().__init__()
        
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.top_layout = QtGui.QGridLayout()
        self.setLayout(self.top_layout)
        self.top_layout.addWidget(self.plot_widget)
        self.resize(850*(1+int(tilt_fig)),700)
        self.setWindowTitle('Putzini')
        self.opts = PutziniConfig()

        # START PLOTS ---
        self.pos_coord = self.plot_widget.addPlot(title="In-plane (color: angle)", row=0, col=0)
        self.pos_coord.setRange(yRange=self.opts.range_y, xRange=self.opts.range_x)
        self.pos_coord.setAspectLocked(True, ratio=1)
        self.pos_coord.setLabel('left', "Y", units='cm')
        self.pos_coord.setLabel('bottom', "X", units='cm')    
        self.all_points = pg.ScatterPlotItem(size=10)
        self.current_point = pg.ScatterPlotItem(size=30)
        self.anchors = {}
        mapimg = imread(self.opts.keepout_img)
        self.map = pg.ImageItem()
        self.map.setImage(image=mapimg.T)
        self.map.setPxMode(False)
        self.map.setRect(QtCore.QRect(-mapimg.shape[1]//2, -mapimg.shape[0]//2, mapimg.shape[0], mapimg.shape[1]))
        self.pos_coord.addItem(self.map)
        for lbl, x, y in zip(self.opts.anchor_names, self.opts.anchor_x, self.opts.anchor_y):
            self.anchors[lbl] = pg.ScatterPlotItem(size=10, symbol='o')
            self.pos_coord.addItem(self.anchors[lbl])
            self.anchors[lbl].addPoints(x=[x], y=[y], brush=None, pen='k', pxMode=False, size=10)
        print(self.anchors)
        self.arka = pg.ScatterPlotItem(size=10, symbol='o', pxMode=False)
        # for lbl, x, y in zip(self.opts.anchor_names, self.opts.anchor_x, self.opts.anchor_y):
        self.pos_coord.addItem(self.current_point)
        self.pos_coord.addItem(self.arka)
        self.pos_coord.addItem(self.all_points)
        self.arka.addPoints(x=[0], y=[0], size=146)

        if tilt_fig:
            self.tilt_coord = self.plot_widget.addPlot(title="Out-of-plane tilt", row=0, col=1)
            self.tilt_coord.setLabel('left', "tilt Y", units='deg')
            self.tilt_coord.setLabel('bottom', "tilt X", units='deg')
            self.tilt_all_points = pg.ScatterPlotItem(size=10)
            self.tilt_current_point = pg.ScatterPlotItem(size=30)
            self.tilt_coord.addItem(self.tilt_current_point)
            self.tilt_coord.addItem(self.tilt_all_points)           
        # STOP PLOTS ---
        
        # START CONTROLS ---
        self.control_layout = QtGui.QFormLayout(verticalSpacing=1)
        
        self.update_graph = QtGui.QCheckBox('Update Graph')
        self.update_graph.setChecked(True)
        self.control_layout.addRow(self.update_graph)
        
        self.keep_traces = QtGui.QCheckBox('Keep Traces')
        self.keep_traces.setChecked(False)
        self.control_layout.addRow(self.keep_traces)        
        
        move_command = QtGui.QLineEdit('stop()')
        move_command.returnPressed.connect(lambda: self.command('move', move_command.text()))
        self.control_layout.addRow(QtGui.QLabel('Move cmd'), move_command)
                
        abs_move = QtGui.QLineEdit('0, 0, 50, 10')
        abs_move.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*-?\d+,\s*\d+')))
        abs_move.returnPressed.connect(lambda: self.command('move', f'moveToPos({abs_move.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Abs move'), abs_move)
                
        rel_move = QtGui.QLineEdit('0, 0, 50, 10')
        rel_move.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*-?\d+,\s*\d+')))
        rel_move.returnPressed.connect(lambda: self.command('move', f'moveByPos({rel_move.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Rel move'), rel_move)
                
        straight_move = QtGui.QLineEdit('0, 50, 10')
        straight_move.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+')))
        straight_move.returnPressed.connect(lambda: self.command('move', f'moveStraight({rel_move.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Straight move'), straight_move)
                
        abs_turn = QtGui.QLineEdit('0, 50')
        abs_turn.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+')))
        abs_turn.returnPressed.connect(lambda: self.command('move', f'moveToAngle({abs_turn.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Abs turn'), abs_turn)
        
        rel_turn = QtGui.QLineEdit('0, 50')
        rel_turn.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+')))
        rel_turn.returnPressed.connect(lambda: self.command('move', f'moveByAngle({rel_turn.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Rel turn'), rel_turn)     
        
        look_at = QtGui.QLineEdit('0, 0, 50')
        look_at.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*-?\d+')))
        look_at.returnPressed.connect(lambda: self.command('move', f'lookAtPos({look_at.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Look at'), look_at)     

        random_rng = QtGui.QLineEdit(f'{self.opts.range_x[0]}, {self.opts.range_x[1]}, {self.opts.range_y[0]}, {self.opts.range_y[1]}')
        random_rng.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*-?\d+,\s*-?\d+')))
        self.control_layout.addRow(QtGui.QLabel('Rnd Range'), random_rng)     
                
        random_bf = QtGui.QLineEdit('20, 15, 50, 1')
        random_bf.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+,\s*\d+,\s*\d+,\s*\d')))
        random_bf.returnPressed.connect(lambda: self.command('move', f'moveBackForth({random_bf.text()},{random_rng.text()})'))
        self.control_layout.addRow(QtGui.QLabel('B-F'), random_bf)     
        
        random = QtGui.QLineEdit('50')
        random.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        random.returnPressed.connect(lambda: self.command('move', f'moveRandom({random.text()},{random_rng.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Zigzag'), random)   
        
        circle = QtGui.QLineEdit('50, 1') # speed, direction
        circle.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+,\s*\d+')))
        circle.returnPressed.connect(lambda: self.command('move', f'moveCircle({circle.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Circle'), circle)   
                
        override = QtGui.QCheckBox('Override Keepout')
        override.stateChanged.connect(lambda state: self.client.m_client.publish('putzini/override', '1' if int(state)==2 else '0'))
        override.setChecked(False)
        self.control_layout.addRow(override)   
                                
        stop = pg.QtGui.QPushButton('STOP')
        stop.clicked.connect(lambda: self.command('move', 'stop()'))
        done = pg.QtGui.QPushButton('Set Ready')
        done.clicked.connect(lambda: self.client.m_client.publish('putzini/force_idle'))
        
        self.control_layout.addRow(stop, done)    
        # stop.setMaximumWidth(200)
        
        vac = pg.QtGui.QCheckBox('Vacuum Cleaner')
        vac.stateChanged.connect(lambda state: self.command('vacuum', '1' if int(state)==2 else '0'))
        vac.setChecked(False)
        self.control_layout.addRow(vac)
        
        height = pg.SpinBox(value=0)
        # height.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        height.setOpts(bounds=(0,130), suffix='mm', step=1, int=True, compactHeight=False)
        height.valueChanged.connect(lambda x: self.command('height', int(x)))
        self.control_layout.addRow(QtGui.QLabel('Height'), height)  
        
        head = pg.SpinBox(value=0)
        # height.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        head.setOpts(bounds=(0,255), suffix='stp', step=1, int=True, compactHeight=False)
        head.valueChanged.connect(lambda x: self.command('head', int(x)))
        self.control_layout.addRow(QtGui.QLabel('Head'), head)  
                                                        
        color_fg = pg.ColorButton()
        color_fg.sigColorChanging.connect(lambda ctrl: self.command('lamp',
            {'front': {'r': ctrl.color().red(), 'g': ctrl.color().green(), 'b': ctrl.color().blue(), 'w': ctrl.color().alpha()}}))
        self.control_layout.addRow(QtGui.QLabel('FG Color'), color_fg)
                 
        color_bg = pg.ColorButton()
        color_bg.sigColorChanging.connect(lambda ctrl: self.command('lamp',
            {'back': {'r': ctrl.color().red(), 'g': ctrl.color().green(), 'b': ctrl.color().blue()}}))
        self.control_layout.addRow(QtGui.QLabel('BG Color'), color_bg)
        
        self.audio_vol = pg.SpinBox(value=10)
        self.audio_vol.setOpts(bounds=(0,150), step=1)
        self.control_layout.addRow(QtGui.QLabel('Volume'), self.audio_vol)
        
        audio_1 = QtGui.QLineEdit('New Peeps/curious_1.wav')
        audio_1.returnPressed.connect(lambda: self.command('audio', {'folder': audio_1.text().rsplit('/', 1)[0], 
                                                                     'file': audio_1.text().rsplit('/', 1)[1], 'loop': 0, 'vol': int(self.audio_vol.value())}
                                                           if audio_1.text() else 'stop'))
        self.control_layout.addRow(QtGui.QLabel('Audio once'), audio_1)
                
        audio_loop = QtGui.QLineEdit('New Peeps/curious_1.wav')
        audio_loop.returnPressed.connect(lambda: self.command('audio', {'folder': audio_loop.text().rsplit('/', 1)[0], 
                                                                     'file': audio_loop.text().rsplit('/', 1)[1], 'loop': 1, 'vol': int(self.audio_vol.value())}
                                                              if audio_loop.text() else 'stop'))
        self.control_layout.addRow(QtGui.QLabel('Audio loop'), audio_loop)
                         
        # indicators
        self.xpos_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('X Position'), self.xpos_indicator)
                 
        self.ypos_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Y Position'), self.ypos_indicator)
                 
        self.angle_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Angle'), self.angle_indicator)
                          
        self.battery_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Battery'), self.battery_indicator)
                          
        self.calibrated_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Calibrated'), self.calibrated_indicator)
        
        random.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        random.returnPressed.connect(lambda: self.command('move', f'moveRandom({random.text()},{random_rng.text()})'))
        
        self.last_err = QtGui.QLineEdit(readOnly=True)
        clear = pg.QtGui.QPushButton('Clear Errors')
        clear.clicked.connect(self.last_err.clear)      
        self.control_layout.addRow(QtGui.QLabel('Last Error'), clear)  
        self.control_layout.addRow(self.last_err)
                
        self.last_cmd = QtGui.QTextEdit(readOnly=True, width=10)
        self.control_layout.addRow(self.last_cmd)        
        self.last_cmd.resize(QtCore.QSize(4,4))
                               
        self.control_layout.setSizeConstraint(self.control_layout.SetFixedSize)
        self.top_layout.addLayout(self.control_layout, 0, 1)
        # END CONTROLS ---
        
        self.client = MqttClient(self)
        self.client.stateChanged.connect(self.on_stateChanged)
        self.client.messageSignal.connect(self.on_messageSignal)
        self.client.voltageSignal.connect(self.on_voltageSignal)
        self.client.distancesSignal.connect(self.on_distancesSignal)
        self.client.calibratedSignal.connect(self.on_calibratedSignal)
        self.client.errorSignal.connect(self.on_errorSignal)
        self.client.command_stateSignal.connect(self.on_command_stateSignal)
        self.client.stateSignal.connect(self.on_stateSignal)
        self.client.logsSignal.connect(self.on_logsSignal)
        
        self.client.hostname = "172.31.1.150"
        self.client.connectToHost()
        
        self.keyPressEvent = self.key_pressed
        
        self.last_RT = None
        self.last_distances = None
        self.last_calibrated = None
        self.step_size = 2000

    def command(self, kind: str, value):
        
        cmd = {kind: value}
        payload = json.dumps(cmd)
        print(payload)
        # print(self.last_cmd.text())
        self.last_cmd.setText(payload)
        self.client.m_client.publish('putzini/commands', payload)
        
    @QtCore.pyqtSlot(int)
    def on_stateChanged(self, state):
        if state == MqttClient.Connected:
            print(state)
            self.client.subscribe("putzini/position")
            self.client.subscribe("putzini/v_batt")
            self.client.subscribe("putzini/error")
            self.client.subscribe("putzini/calibrated")
            self.client.subscribe("putzini/distances")
            self.client.subscribe("putzini/command_state")
            self.client.subscribe("putzini/state")
            self.client.subscribe("putzini/logs")
            

    @QtCore.pyqtSlot(dict)
    def on_stateSignal(self, state: dict):
        # print('State', state)
        if 'action' in state: 
            if state["action"].lower() in ['error', 'errordontstop']:
                self.setWindowTitle('Putzini (ERROR ERROR ERROR ERROR)')
            elif state["action"].lower() == 'moving':
                self.setWindowTitle('Putzini (moving)')
            elif state["action"].lower() == 'idle':
                self.setWindowTitle('Putzini (idle)')
            else:
                self.setWindowTitle('Putzini (unknown move state)')
                
        # self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(str)
    def on_logsSignal(self, log_message: str):
        print(log_message)
        # self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(dict)
    def on_command_stateSignal(self, command_state: dict):
        print('Command state', command_state)
        # self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(float)
    def on_voltageSignal(self, voltage):
        self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(tuple)
    def on_calibratedSignal(self, calibrated):
        self.calibrated_indicator.setText(f'S{calibrated[0]} G{calibrated[1]} A{calibrated[2]} M{calibrated[3]}')
        # self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(PutziniError)
    def on_errorSignal(self, error):
        self.last_err.setText(f'{error}')
        print(error)

    @QtCore.pyqtSlot(dict)
    def on_distancesSignal(self, distances):
        # print(f'Distances: {distances}')
        dist = np.array(distances['d'])*100
        # num = np.array(distances['N'])
        # if (self.last_distances is None) or not np.allclose(distances['d'], self.last_distances):
        if True:
            self.last_distances = dist
            
            if self.update_graph.isChecked():        
                plot_dat = []
                for lbl, x, y, d, fac in zip(self.opts.anchor_names, self.opts.anchor_x, self.opts.anchor_y, 
                                             dist, self.opts.distance_factors):
                # self.anchors.clear()   
                    self.anchors[lbl].setData(x=[x], y=[y], brush=None, 
                                              pen=pg.mkPen((200,100,100,90), width=6), 
                                              pxMode=False, size=2*fac*d)
                    
                #     plot_dat.append({'pos': (x, y),
                #                     'size': d,
                #                     'symbol': 'o',
                #                     'name': name})
                
                # self.anchors.setData(spots=plot_dat, pxMode=False)
        
        # self.battery_indicator.setText(f'{voltage:.2f} V')

    @QtCore.pyqtSlot(str)
    def on_messageSignal(self, msg):
        
        RT = eval('np.' + msg.replace('nan', 'np.nan'))

        if (self.last_RT is None) or not np.allclose(RT, self.last_RT):
            
            # d_T = sum((RT - self.last_RT if self.last_RT is None else 0)[-3:-1]**2)**.5
            d_T = np.inf if self.last_RT is None else sum((RT - self.last_RT)[:3,-1]**2)**.5
            self.last_RT = RT
            T = RT[:3,-1]
            angles = (Rotation.from_matrix(RT[:3,:3]).as_euler('XYZ')*180/np.pi).round(1)
        
            self.xpos_indicator.setText(f'{T[0]*100:.0f} cm')
            self.ypos_indicator.setText(f'{T[1]*100:.0f} cm')
            self.angle_indicator.setText(f'{angles[0]},{angles[1]},{angles[2]}')
                            
            if self.update_graph.isChecked():
                                
                inplane = angles[2]
                # print(pg.hsvColor(angles[2]/np.pi).getRgb())
                tr = QtGui.QTransform()
                angle_rot = tr.rotate(-inplane + 180)
                my_rotated_symbol = angle_rot.map(my_symbol)
                col = pg.hsvColor((inplane+180)/360)
            
                with np.printoptions(precision=2, suppress=True):
                    # print(f'Pos: {T}, Euler XYZ: {(angles)}, In-plane: {inplane}')
                    pass
                    
                if self.keep_traces.isChecked():
                    self.all_points.addPoints([{'pos': T[:-1]*100, 'data': 1, 'brush': None, 'pen': pg.mkPen(col),
                                                'symbol': 'o', 'size': 5}])
                else:
                    self.all_points.clear()
                
                # self.current_point.clear()
                self.current_point.setData([{'pos': T[:-1]*100, 'data': 1, 'brush':pg.mkBrush(col), 
                                            'symbol': my_rotated_symbol, 'size': 30}])    
                                                    
                if tilt_fig:
                    # self.tilt_current_point.clear()
                    self.tilt_current_point.setData([{'pos': angles[:2], 'data': 1, 'brush':pg.mkBrush(col), 
                                                'size': 30}], clear=True)
                    self.tilt_all_points.addPoints([{'pos': angles[:2], 'data': 1, 'brush':pg.mkBrush(col),
                                                'size': 10}])          
                    
                QtGui.QGuiApplication.processEvents()

    def key_pressed(self, ev):
        k = ev.key()
        qtk = pg.Qt.QtCore.Qt
        if k == qtk.Key_Up:
            print('Up')
            self.client.m_client.publish('putzini/move', -self.step_size)
        
        elif k == qtk.Key_Down:
            print('Down')
            self.client.m_client.publish('putzini/move', self.step_size)
            
        elif k == qtk.Key_Left:
            print('Left')      
            self.client.m_client.publish('putzini/turn', self.step_size//5)
                              
        elif k == qtk.Key_Right:
            print('Right')       
            self.client.m_client.publish('putzini/turn', -self.step_size//5)
            
        elif k == qtk.Key_W:           
            self.step_size += 200
            print('New step size is', self.step_size)
            
        elif k == qtk.Key_R:
            self.all_points.clear()
            if tilt_fig:
                self.tilt_all_points.clear()
            QtGui.QGuiApplication.processEvents()
            print('Cleared plot')            
            
        elif k == qtk.Key_S:   
            self.step_size -= 200    
            print('New step size is', self.step_size)
            
        elif k == qtk.Key_Space:   
            self.command('move', 'stop()') 
            print('Movement stopped')
                                    
        else:
            # pg.Qt.Key
            print('Key pressed:', k)

if __name__ == '__main__':
    import sys
    app = QtGui.QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())

