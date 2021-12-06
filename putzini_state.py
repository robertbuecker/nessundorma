import asyncio_mqtt as mqtt
import asyncio
import numpy as np
import json
import logging
from putzini_keepout import KeepoutError, PathForbiddenError
logger = logging.getLogger(__name__)

class PutziniState:
    def __init__(self, mqtt_client):
        self.logger = logger
        self.mqtt_client = mqtt_client
        self.pos = np.zeros(3)
        self.alpha = np.array([0., 0., 0.])
        self.action = 'Idle'
        self.message = ''
    
    def set_active(self):
        self.action = 'Moving'
        self.publish(qos=1)
        
    def set_idle(self, move_task=None):
        # called whenever a motion task is finished
        if isinstance(move_task, asyncio.Future):
            self.logger.debug('Received to idle with task %s', move_task)
            try:
                raise move_task.exception()
            except TypeError:
                # there was no exception, all is good
                pass
            except asyncio.CancelledError as exc:
                self.logger.info('Task %s was cancelled, i.e. replaced by another. Not setting idle.', move_task)
                return
            except PathForbiddenError as err:
                self.logger.warning('Movement from task %s is forbidden: %s.', move_task, err)
            except KeepoutError as err:
                self.logger.critical('Keepout violation during task %s: %s', move_task, err)
                self.set_error(str(err))
                return
            except Exception as err:
                self.logger.error('Weird error during task %s: %s', move_task, err)
        elif move_task is None:
            self.logger.warning('Received set_idle without task. Probably a manual stop().')

        self.action = "Idle"
        self.message = ''
        self.publish(qos=1)

    def set_error(self, message=''):
        self.action = 'Error'
        # self.action = 'ErrorDontStop'
        self.message = message
        # TODO to state the obvious...
        self.logger.critical('Putzini set to error state!')
        self.publish(qos=1)        
        
    def set_position_with_alpha(self, pos, alpha):
        self.pos = pos
        self.alpha = alpha 
        self.publish(qos=0)
        
    def publish(self, qos=0):
        asyncio.ensure_future(self.mqtt_client.publish("putzini/state", json.dumps({'action': self.action, 
            'posX': self.pos[0], 'posY': self.pos[1], 'alpha': self.alpha[0], 'message': self.message}), qos=qos))  
