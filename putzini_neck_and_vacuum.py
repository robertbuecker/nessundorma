import asyncio
import serial_asyncio
import struct


class PutziniNeckAndVacuum:
    def __init__(self):
        self.current_pos = 0
        self.calibrated = False
        self.speed = 127
        self.vacuum = 0
        self.aux = 0
        pass
  
    async def connect(self, url="/dev/serial/by-path/platform-70090000.xusb-usb-0:2.1:1.0-port0", baudrate=115200):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=baudrate)
        await asyncio.sleep(0.8)
        self.set_aux(0)
        await asyncio.sleep(3)
        self.move(-30000)
        self.calibrated = True

    def move(self, steps , speed = 250):
        self.speed = speed
        frame = struct.pack(">lBB",int(steps), self.speed, self.vacuum)
        self.writer.write(frame)

    def set_position(self, position):
        #convert to steps 
        position = float(position)/0.005
        if self.calibrated:
            self.move(position-self.current_pos)
            self.current_pos = position
        else:
            print("Error: Neck not calibrated")
        
    def set_zero(self, _ ):
        self.current_pos = 0
        self.calibrated = True
        
    def set_vacuum(self, onoff):
        """0 == off, 1 == on"""
        
        self.vacuum = int(onoff)
        frame = struct.pack(">lBB",0, self.speed, self.vacuum)
        self.writer.write(frame)
        
    def set_aux(self, onoff):
        self.aux = int(onoff)
        frame = struct.pack(">lBBB",0, self.speed, self.vacuum, self.aux)
        self.writer.write(frame)
        
        
if __name__ == '__main__':
    async def main():
        pnv = PutziniNeckAndVacuum()
        await pnv.connect()
        await asyncio.sleep(10)
        pnv.set_vacuum(1)
        while True:
            asyncio.sleep(10)
    
    loop = asyncio.get_event_loop()      
    loop.set_debug(True)
    loop.run_until_complete(main())
    loop.close()
