
#typedef struct{
#   uint16_t start;
#   int16_t 	cmd1;
#   int16_t 	cmd2;
#   int16_t 	speedR_meas;
#   int16_t 	speedL_meas;
#   int16_t 	batVoltage;
#   int16_t 	boardTemp;
#   uint16_t cmdLed;
#   uint16_t checksum;
#} SerialFeedback;

import asyncio
import serial_asyncio
import asyncio_mqtt as mqtt
import struct
import logging

logger = logging.getLogger(__name__)

class PutziniDrive:

    START_FRAME = 0xABCD
    
    def __init__(self, mqtt_client):
        self.logger = logger
        self.mqtt_client = mqtt_client
    
        self.meas_speed_l = 0
        self.meas_speed_r = 0
        self.bat_voltage = 0
        self.temperature = 0
        
        self.speed_l = 0
        self.speed_r = 0
        
        self.distance_to_move_l = 0
        self.distance_to_move_r = 0
        
        self.read_error = True
        
        self.loop = asyncio.get_event_loop()
        self.finished = self.loop.create_future()
        self.finished.set_result(None)
        self.period = 50e-3
        self.wheel_balance = 1.02 # larger: rather move to the left

    async def connect(self, url='/dev/serial/by-path/platform-70090000.xusb-usb-0:2.4:1.0-port0', baudrate=38400):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)       
        asyncio.ensure_future(self._reader_task())
        asyncio.ensure_future(self._writer_task())
         
    async def _reader_task(self):
        msg=b''
        i = 0
        while True:
            msg = await self.reader.read(18)
            if len(msg) == 18:
                msg=struct.unpack('HhhhhhhHH',msg)
                if msg[0] == PutziniDrive.START_FRAME and msg[8] == (msg[0] ^ msg[1] ^ msg[2] ^ msg[3] ^ msg[4] ^ msg[5] ^ msg[6] ^ msg[7]) & 0xffff:
                    self.read_error = False
                    self.meas_speed_l = msg[4]*2
                    self.meas_speed_r = msg[3]*-2
                    self.bat_voltage = msg[5]/100
                    self.temperature = msg[6]/10
                    
                    self.distance_to_move_l -= abs(self.meas_speed_l)
                    self.distance_to_move_r -= abs(self.meas_speed_r)
                    
                    i += 1
                    if i > 200:
                        asyncio.ensure_future(self.mqtt_client.publish("putzini/v_batt", self.bat_voltage, qos=0))
                        asyncio.ensure_future(self.mqtt_client.publish("putzini/temp", self.temperature, qos=0))
                        i=0
                    
                    #print(f"cmd1 {msg[1]}, cmd2: {msg[2]}, spdR:{msg[3]}, spdL:{msg[4]}, batVoltage: {msg[5]/100}, temp: {msg[6]/10}")
                else:
                    self.read_error = True
	                
    async def _writer_task(self):
        while True:
            await asyncio.sleep(self.period)
            speed_r = 0
            speed_l = 0
            
            if not self.read_error and self.distance_to_move_l > 0:
                speed_l = self.speed_l
                
            if not self.read_error and self.distance_to_move_r > 0:
                speed_r = self.speed_r
            
            if not self.read_error and self.distance_to_move_r <= 0 and self.distance_to_move_l <= 0:
                if not self.finished.done():
                    self.finished.set_result(None) 
            
            # set
            self.moving = speed_r > 0 or speed_l > 0
            
            # WHEEL FUDGE FACTOR
            # for motor imbalance. Note that "r" means the LEFT wheel seen from behind!
            speed_l = int(speed_l * self.wheel_balance)
            speed_r = int(speed_r * 1/self.wheel_balance)
        
            frame = struct.pack('HhhH',PutziniDrive.START_FRAME, speed_r, speed_l, (PutziniDrive.START_FRAME ^ speed_r ^ speed_l) & 0xffff)
            self.writer.write(frame)

    def stop(self):
        if (self.distance_to_move_l != 0) or (self.distance_to_move_r != 0):
            self.logger.info('Stop at remaining distances R=%s, L=%s', self.distance_to_move_r, self.distance_to_move_l)
        self.distance_to_move_r = 0
        self.distance_to_move_l = 0


    def move_r(self, dist):
        self.distance_to_move_r = abs(int(dist))
        if self.finished.done():
            self.finished = self.loop.create_future()
    
    def move_l(self,dist):
        self.distance_to_move_l = abs(int(dist))
        if self.finished.done():
            self.finished = self.loop.create_future()
        
    def set_speed_l(self, speed):
        self.speed_l = int(speed)
        
    def set_speed_r(self, speed):
        self.speed_r = int(speed)
        
    def turn(self, angle, speed=60):
        if int(angle) < 0:
            speed*=-1
        self.set_speed_l(-int(speed))
        self.set_speed_r(int(speed))
        self.move_r(int(angle))
        self.move_l(int(angle))
        if self.finished.done():
            self.finished = self.loop.create_future()
            
    def move(self, dist, speed=60):
        if int(dist) < 0:
            speed *=-1
        self.set_speed_r(speed)
        self.set_speed_l(speed)
        self.distance_to_move_l = abs(int(dist))
        self.distance_to_move_r = abs(int(dist))
        if self.finished.done():
            self.finished = self.loop.create_future()

