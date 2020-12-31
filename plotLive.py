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

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

import paho.mqtt.client as mqtt

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
        mstr = msg.payload.decode("utf-8")
        # print("on_message", mstr, obj, mqttc)
        self.messageSignal.emit(mstr)

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

        self.pos_coord = self.plot_widget.addPlot(title="In-plane (color: angle)", row=0, col=0)
        self.pos_coord.setLabel('left', "Y", units='m')
        self.pos_coord.setLabel('bottom', "X", units='m')
        
        self.all_points = pg.ScatterPlotItem(size=10)
        self.current_point = pg.ScatterPlotItem(size=30)
        self.pos_coord.addItem(self.current_point)
        self.pos_coord.addItem(self.all_points)

        if tilt_fig:
            self.tilt_coord = self.plot_widget.addPlot(title="Out-of-plane tilt", row=0, col=1)
            self.tilt_coord.setLabel('left', "tilt Y", units='deg')
            self.tilt_coord.setLabel('bottom', "tilt X", units='deg')
            self.tilt_all_points = pg.ScatterPlotItem(size=10)
            self.tilt_current_point = pg.ScatterPlotItem(size=30)
            self.tilt_coord.addItem(self.tilt_current_point)
            self.tilt_coord.addItem(self.tilt_all_points)           
        
        self.control_layout = QtGui.QFormLayout()
               
        def haveacolor(ctrl):
            print(ctrl.color().getRgb())
        
        self.update_graph = QtGui.QCheckBox('Update Graph')
        self.update_graph.setChecked(True)
        self.control_layout.addRow(self.update_graph)
        
        move_command = QtGui.QLineEdit('stop()')
        move_command.returnPressed.connect(lambda: self.command('move', move_command.text()))
        self.control_layout.addRow(QtGui.QLabel('Move cmd'), move_command)
                
        abs_move = QtGui.QLineEdit('0, 0, 0')
        abs_move.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*\d+')))
        abs_move.returnPressed.connect(lambda: self.command('move', f'moveToPos({abs_move.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Abs move'), abs_move)
                
        rel_move = QtGui.QLineEdit('0, 0, 0')
        rel_move.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+,\s*\d+')))
        rel_move.returnPressed.connect(lambda: self.command('move', f'moveByPos({rel_move.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Rel move'), rel_move)
        
        abs_turn = QtGui.QLineEdit('0, 0')
        abs_turn.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+')))
        abs_turn.returnPressed.connect(lambda: self.command('move', f'moveToAngle({abs_turn.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Abs turn'), abs_turn)
        
        rel_turn = QtGui.QLineEdit('0, 0')
        rel_turn.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('-?\d+,\s*-?\d+')))
        rel_turn.returnPressed.connect(lambda: self.command('move', f'moveByAngle({rel_turn.text()})'))
        self.control_layout.addRow(QtGui.QLabel('Rel turn'), rel_turn)     
        
        turn_to_arka = pg.QtGui.QPushButton('To Arka')
        turn_to_arka.clicked.connect(lambda: self.command('move', 'lookAtArka()'))
        self.control_layout.addRow(turn_to_arka)     
        turn_to_arka.setMaximumWidth(200)
        
        stop = pg.QtGui.QPushButton('STOP')
        stop.clicked.connect(lambda: self.command('move', 'stop()'))
        self.control_layout.addRow(stop)    
        stop.setMaximumWidth(200)
        
        height = pg.SpinBox(value=0)
        # height.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        height.setOpts(bounds=(0,100), suffix='mm', step=1, int=True, compactHeight=False)
        height.valueChanged.connect(lambda x: self.command('height', int(x)))
        self.control_layout.addRow(QtGui.QLabel('Height'), height)  
        
        head = pg.SpinBox(value=0)
        # height.setValidator(QtGui.QRegExpValidator(QtCore.QRegExp('\d+')))
        head.setOpts(bounds=(0,255), suffix='stp', step=1, int=True, compactHeight=False)
        head.valueChanged.connect(lambda x: self.command('height', int(x)))
        self.control_layout.addRow(QtGui.QLabel('Head'), head)  
                                                        
        color_fg = pg.ColorButton()
        color_fg.sigColorChanging.connect(lambda ctrl: self.command('lamp',
            {'front': {'r': ctrl.color().red(), 'g': ctrl.color().green(), 'b': ctrl.color().blue(), 'w': ctrl.color().alpha()}}))
        self.control_layout.addRow(QtGui.QLabel('FG Color'), color_fg)
                 
        color_bg = pg.ColorButton()
        color_bg.sigColorChanging.connect(lambda ctrl: self.command('lamp',
            {'back': {'r': ctrl.color().red(), 'g': ctrl.color().green(), 'b': ctrl.color().blue()}}))
        self.control_layout.addRow(QtGui.QLabel('BG Color'), color_bg)
                 
        self.xpos_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('X Position'), self.xpos_indicator)
                 
        self.ypos_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Y Position'), self.ypos_indicator)
                 
        self.angle_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Angle'), self.angle_indicator)
                          
        self.battery_indicator = QtGui.QLineEdit(readOnly=True)
        self.control_layout.addRow(QtGui.QLabel('Bat Volt'), self.battery_indicator)
                               
        self.control_layout.setSizeConstraint(self.control_layout.SetFixedSize)
        # self.control_layout.SetMaximumSize(100,1000)
        # self.control_layout.resize(100, self.control_layout.width())
        self.top_layout.addLayout(self.control_layout, 0, 1)
        

        self.client = MqttClient(self)
        self.client.stateChanged.connect(self.on_stateChanged)
        self.client.messageSignal.connect(self.on_messageSignal)

        self.client.hostname = "192.168.1.19"
        self.client.connectToHost()
        
        self.keyPressEvent = self.key_pressed
        
        self.last_RT = None
        self.step_size = 500

    def command(self, kind: str, value):
        
        cmd = {kind: value}
        payload = json.dumps(cmd)
        print(payload)
        self.client.m_client.publish('putzini/commands', payload)
        
    @QtCore.pyqtSlot(int)
    def on_stateChanged(self, state):
        if state == MqttClient.Connected:
            print(state)
            self.client.subscribe("putzini/position")

    @QtCore.pyqtSlot(str)
    def on_messageSignal(self, msg):
        RT_in = eval('np.' + msg)
        # RT_in = np.dot(np.diag([1,-1,-1,1]), RT_in)
        # angles = Rotation.from_dcm(RT_in[:3, :3]).as_euler('XYZ')
        # R0 = Rotation.from_euler('XYZ', angles*np.array([0,0,1])).as_dcm()
        # RT_in[:3,:3] = R0
        # RT = np.linalg.inv(RT_in)
        # RT[:2,:2] = np.dot([[0, 1], [-1, 0]], RT[:2,:2])
        # RT = np.linalg.inv(RT)
        RT = RT_in
        # RT[:2,:2] = np.dot([[0, 1], [-1, 0]], RT[:2,:2])
        
        if (self.last_RT is None) or (self.last_RT != RT).any():
            
            self.last_RT = RT
            T = RT[:3,-1]
            angles = (Rotation.from_dcm(RT[:3,:3]).as_euler('ZYX')*180/np.pi).round(1)
        
            self.xpos_indicator.setText(f'{T[0]*100:.0f} cm')
            self.ypos_indicator.setText(f'{T[1]*100:.0f} cm')
            self.angle_indicator.setText(f'{angles[0]},{angles[1]},{angles[2]}')
            
            
            with np.printoptions(precision=2, suppress=True):
                print(f'New position: {T}, angles (Euler ZYX, deg): {(angles*180/np.pi)}')
                            
            if self.update_graph.isChecked():
                inplane = angles[0]
                # print(pg.hsvColor(angles[2]/np.pi).getRgb())
                tr = QtGui.QTransform()
                angle_rot = tr.rotate(-inplane + 180)
                my_rotated_symbol = angle_rot.map(my_symbol)
                col = pg.hsvColor((inplane+180)/360)
                self.all_points.addPoints([{'pos': T[:-1], 'data': 1, 'brush': None, 'pen': pg.mkPen(col),
                                            'symbol': my_rotated_symbol, 'size': 10}])
                self.current_point.clear()
                self.current_point.addPoints([{'pos': T[:-1], 'data': 1, 'brush':pg.mkBrush(col), 
                                            'symbol': my_rotated_symbol, 'size': 30}])    
                if tilt_fig:
                    self.tilt_current_point.clear()
                    self.tilt_current_point.addPoints([{'pos': angles[-1:0:-1]*180/np.pi, 'data': 1, 'brush':pg.mkBrush(col), 
                                                'size': 30}])
                    self.tilt_all_points.addPoints([{'pos': angles[-1:0:-1]*180/np.pi, 'data': 1, 'brush':pg.mkBrush(col),
                                                'size': 10}])                

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
            print('PgUp')                 
            self.step_size += 200
            print('New step size is', self.step_size)
            
        elif k == qtk.Key_R:
            print('R')
            self.all_points.clear()
            if tilt_fig:
                self.tilt_all_points.clear()
            print('New step size is', self.step_size)            
            
        elif k == qtk.Key_S:   
            print('PgDn')   
            self.step_size -= 200    
            print('New step size is', self.step_size)
                        
        else:
            # pg.Qt.Key
            print('Key pressed:', k)

if __name__ == '__main__':
    import sys
    app = QtGui.QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())

