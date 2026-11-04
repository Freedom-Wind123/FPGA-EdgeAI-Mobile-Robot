"""Task/observation protocol v1. No automatic arming, and no motor PWM API.

Keep one process owning the port. The old ROS ASCII velocity sender MUST be
stopped when using this firmware. Telemetry remains the existing ASCII format.
"""
import math
import struct
from enum import IntEnum

class Kind(IntEnum):
    HEARTBEAT=1
    STOP=2
    ARM=3
    MAP_GOAL=0x10
    FOLLOW=0x11
    TARGET=0x12
    MANUAL=0x13

def crc16(data):
    crc=0xffff
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc=((crc << 1)^0x1021 if crc & 0x8000 else crc << 1) & 0xffff
    return crc

def q16(value):
    if not math.isfinite(value):
        raise ValueError('non-finite value')
    n=round(value*65536)
    if not -(1<<31) <= n < (1<<31):
        raise ValueError('Q16 overflow')
    return n

def encode(kind,sequence,payload=b''):
    if len(payload)>24 or not 0<=sequence<=65535:
        raise ValueError('bad frame length/sequence')
    body=struct.pack('<BBH',1,int(kind),sequence)+payload.ljust(24,b'\0')
    return b'\xaa\x55'+body+struct.pack('<H',crc16(body))

def decode(frame):
    if len(frame)!=32 or frame[:3]!=b'\xaa\x55\x01':
        raise ValueError('bad header')
    if crc16(frame[2:30])!=struct.unpack('<H',frame[30:])[0]:
        raise ValueError('bad CRC')
    return Kind(frame[3]),struct.unpack('<H',frame[4:6])[0],frame[6:30]

class TaskLink:
    """writer is a serial-like write(bytes) object; caller owns heartbeat loop.

    Sequence must be persisted by the sole writer or board reset at reconnect.
    Never automatically resume an old task after disconnect. STOP is always
    accepted even if sequence is old; subsequent tasks still need a newer seq.
    """
    def __init__(self,writer,sequence=0):
        self.writer=writer
        self.sequence=sequence

    def send(self,kind,payload=b''):
        frame=encode(kind,self.sequence,payload)
        self.sequence=(self.sequence+1)&65535
        result=self.writer.write(frame)
        if result != len(frame):
            raise IOError('short UART write; task was not delivered completely')
        return frame

    def stop(self):return self.send(Kind.STOP)
    def heartbeat(self):return self.send(Kind.HEARTBEAT)
    def arm(self):return self.send(Kind.ARM)
    def map_goal(self,x_m,y_m):
        if not all(math.isfinite(v) and -7.5<=v<=7.5 for v in (x_m,y_m)):
            raise ValueError('goal outside map')
        return self.send(Kind.MAP_GOAL,struct.pack('<ii',q16(x_m),q16(y_m)))
    def follow(self):return self.send(Kind.FOLLOW)
    def target(self,bearing_rad,range_mm,valid=True):
        if not valid:return self.send(Kind.TARGET,b'\0'*9)
        if not math.isfinite(bearing_rad) or abs(bearing_rad)>math.pi/2:
            raise ValueError('invalid target bearing')
        if not isinstance(range_mm,int) or not 100<=range_mm<=12000:
            raise ValueError('target range needs a calibrated estimate in mm')
        return self.send(Kind.TARGET,struct.pack('<iiB',q16(bearing_rad),range_mm,1))
    def manual(self,vx,vy,wz):
        if not all(math.isfinite(v) and abs(v)<=cap for v,cap in zip((vx,vy,wz),(.149,.149,.299))):
            raise ValueError('manual speed exceeds development cap')
        return self.send(Kind.MANUAL,struct.pack('<iii',q16(vx),q16(vy),q16(wz)))

def bearing_from_pixel(cx,width,hfov_deg):
    """Pinhole conversion. Positive bearing is left, positive image x is right.

    hfov must come from camera calibration, not an assumed camera model.
    """
    if not all(math.isfinite(v) for v in (cx,width,hfov_deg)) or not width>0 or not 0<hfov_deg<180 or not 0<=cx<=width:
        raise ValueError('invalid camera geometry')
    focal=width/(2*math.tan(math.radians(hfov_deg)/2))
    return math.atan((width/2-cx)/focal)
