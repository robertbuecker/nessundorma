import asyncio_mqtt as mqtt
import asyncio
import numpy as np
import json

class PutziniState:
    def __init__(self, mqtt_client):
        self.mqtt_client = mqtt_client
        self.pos = np.zeros(3)
        self.alpha = np.array([0., 0., 0.])
        self.action = 'Idle'
    
    def set_active(self):
        self.action = 'Moving'
        self.publish()
        
    def set_idle(self, move_task=None):
        if (move_task is not None) and (not isinstance(move_task, str)) and (move_task.exception() is not None):
            # raise move_task.exception()
            if not isinstance(move_task.exception(), asyncio.CancelledError):
                print(move_task.exception())
                self.set_error()
        else:
            self.action = "Idle"
            self.publish()

    def set_error(self):
        self.action = 'Error'
        self.publish()        
        
    def set_position_with_alpha(self, pos, alpha):
        self.pos = pos
        self.alpha = alpha 
        self.publish()
        
    def publish(self):
        asyncio.ensure_future(self.mqtt_client.publish("putzini/state", json.dumps({'action': self.action, 'posX': self.pos[0], 'posY': self.pos[1], 'alpha': self.alpha[0]}), qos=0))  
