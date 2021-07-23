#!/usr/bin/env python3

import asyncio
import serial_asyncio
import struct
import random
import colorsys
import math


class PutziniLamp:

    class ContinuesLightIterator:
        def __init__(self,l):
            self.l = l
        def __next__(self):
            frame = struct.pack("BBB",self.l["back"]["r"],self.l["back"]["g"],self.l["back"]["b"])
            for x in range(0,12):
                frame += struct.pack("BBBB",self.l["front"]["r"],self.l["front"]["g"],self.l["front"]["b"],self.l["front"]["w"])
            return frame
            
    class BlinkOnceLightIterator:
        def __init__(self,l):
            self.l = l
            self.first = True
            self.second = True
        def __next__(self):
            if self.first or (not self.first and not self.second):
                self.first = False            
                frame = struct.pack("BBB",self.l["back"]["r"],self.l["back"]["g"],self.l["back"]["b"])
                for x in range(0,12):
                    frame += struct.pack("BBBB",self.l["front"]["r"],self.l["front"]["g"],self.l["front"]["b"],self.l["front"]["w"])
            if not self.first and self.second:
                self.second = False
                frame = b"\x00"*(3+4*13)
            return frame
    
    class BlinkLightIterator:
        def __init__(self,l):
            self.l = l
            self.even = True
        def __next__(self):
            self.even = not self.even  
            if self.even:          
                frame = struct.pack("BBB",self.l["back"]["r"],self.l["back"]["g"],self.l["back"]["b"])
                for x in range(0,12):
                    frame += struct.pack("BBBB",self.l["front"]["r"],self.l["front"]["g"],self.l["front"]["b"],self.l["front"]["w"])
            else:
                self.second = False
                frame = b"\x00"*(3+4*13)
            return frame
    
            
    class RoundLightIterator:
        def __init__(self,l):
            self.l = l
            self.i = 0
        def __next__(self):
            self.i += 1
            if self.i > 12:
                self.i=1
            frame = struct.pack("BBB",self.l["back"]["r"],self.l["back"]["g"],self.l["back"]["b"])
            
            front_color = colorsys.rgb_to_hls(self.l["front"]["r"]/255,self.l["front"]["g"]/255,self.l["front"]["b"]/255)
            
            for x in range(0,12):
                color =  colorsys.hls_to_rgb(front_color[0],front_color[1]*(math.cos((x+self.i)*2*math.pi/12)+1)/2,front_color[2])        
                frame += struct.pack("BBBB",int(color[0]*255),int(color[1]*255),int(color[2]*255),self.l["front"]["w"])
            return frame
            
    class RandomLightIterator:
        def __init__(self,l):
            self.l = l
            if not "back" in l:
                self.l["back"] = {"l":127,"s":255}
            if not "front" in l:
                self.l["front"] = {"l":1,"s":255}
                
        def __next__(self):
            color =  colorsys.hls_to_rgb(random.random(), self.l["back"]["l"]/255, self.l["back"]["s"]/255)
            frame = struct.pack("BBB",int(color[0]*255),int(color[1]*255),int(color[2]*255))
            for x in range(0,12):
                color =  colorsys.hls_to_rgb(random.random(), self.l["front"]["l"]/255, self.l["front"]["s"]/255)
                frame += struct.pack("BBBB",int(color[0]*255),int(color[1]*255),int(color[2]*255),0)
            return frame    
            

    def __init__(self):
        self._i = PutziniLamp.BlinkLightIterator({"back":{"r":255,"g":255,"b":100,"l":1,"s":255},"front":{"r":0,"g":24,"b":0,"w":0,"l":1,"s":255}})
        self.h = 127
        self.delay = 1000e-3
        self.effects = {"random":PutziniLamp.RandomLightIterator, "round":PutziniLamp.RoundLightIterator,
                       "blink":PutziniLamp.BlinkLightIterator, "blinkOnce":PutziniLamp.BlinkOnceLightIterator,
                       "continues":PutziniLamp.ContinuesLightIterator}
        pass
    
    async def connect(self, url = '/dev/serial/by-path/platform-70090000.xusb-usb-0:2.3:1.0-port0'):
        _ , self.writer = await serial_asyncio.open_serial_connection(url=url, baudrate=460800)
        asyncio.ensure_future(self._writer_task())
    
    async def _writer_task(self):
        while True:
            frame = struct.pack("B", self.h)
            frame += self._i.__next__()
            self.writer.write(frame)
            await asyncio.sleep(self.delay)
    
    def set_lamp(self,l):
        if "effect" in l and l["effect"] in self.effects:
            self._i = self.effects[l["effect"]](l)
        else:
            self._i = self.effects["continues"](l)
        if "freq" in l:
            self.delay = 1/l["freq"]
        else:
            self.delay = 100e-3
    
    def set_head(self, h):
        print('Trying to set head to', h)
        self.h = int(h)
        
        

if __name__ == '__main__':

    async def main():
        l = PutziniLamp()
        await l.connect("/dev/ttyUSB0")
        l.set_lamp({"effect":"round","freq":120,"back":{"r":255,"g":255,"b":100,"l":1,"s":255},"front":{"r":0,"g":24,"b":0,"w":0,"l":127,"s":255}})
        while True:
            await asyncio.sleep(1)
        
    loop = asyncio.get_event_loop()      
    loop.run_until_complete(main())
    loop.close()
