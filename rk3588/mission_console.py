"""Development task console. Default: DRY RUN. No automatic arm or task replay.

Commands: arm, stop, goal X Y, follow CLASS COLOR, quit
follow uses numeric model class ID/color ID and calibrated monocular range.
This does not start/replace existing ROS or voice services.
"""
import argparse,json,math,os,queue,sys,threading,time
from pathlib import Path
from fpga_link import TaskLink,Kind

class Observations:
    def __init__(self):self.timestamp=-1;self.track=None
    def select(self,data,now_ms,cls,color):
        ts=data.get('timestamp_ms',-1)
        if not isinstance(ts,(int,float)) or not math.isfinite(ts) or ts<=self.timestamp or not 0<=now_ms-ts<=150:
            return None
        self.timestamp=ts
        matches=[]
        for o in data.get('objects',[]):
            try:
                if o['cls']!=cls or (color and o['color']!=color) or o['confidence']<.6:continue
                if self.track is not None and o['id']!=self.track:continue
                if not all(math.isfinite(o[k]) for k in ['confidence','bearing_rad','range_mm']):continue
                if abs(o['bearing_rad'])>math.pi/2 or not 100<=o['range_mm']<=12000:continue
                matches.append(o)
            except (KeyError,TypeError,ValueError):continue
        if not matches:return None
        result=max(matches,key=lambda o:o['confidence'])
        self.track=result['id']
        return result

class Writer:
    def __init__(self,serial,sequence_path):self.serial=serial;self.path=sequence_path
    def write(self,frame):
        # Reserve next sequence BEFORE transmitting, so a crash cannot replay.
        seq=int.from_bytes(frame[4:6],'little')
        temp=self.path.with_suffix('.tmp')
        with temp.open('w',encoding='ascii') as f:
            f.write(str((seq+1)&65535));f.flush();os.fsync(f.fileno())
        temp.replace(self.path)
        if self.serial:return self.serial.write(frame)
        print('DRY',Kind(frame[3]).name,flush=True)
        return len(frame)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port',help='Actual serial port; omitted means dry-run')
    ap.add_argument('--observations',default='/tmp/fpga_observations.json')
    ap.add_argument('--sequence',default=str(Path.home()/'.fpga_task_sequence'))
    ap.add_argument('--camera-calibrated',action='store_true',help='Confirm actual HFOV, focal length and target size were calibrated')
    args=ap.parse_args()
    serial=None;lock=None
    if args.port:
        import fcntl
        import serial as pyserial
        lock=open('/tmp/fpga_mission_console.lock','w')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        serial=pyserial.Serial(args.port,115200,timeout=0,write_timeout=.2,exclusive=True)
    seq_path=Path(args.sequence)
    seq=int(seq_path.read_text()) if seq_path.exists() else 0
    link=TaskLink(Writer(serial,seq_path),seq)
    commands=queue.Queue()
    def reader():
        for line in sys.stdin:commands.put(line.strip())
        commands.put('quit')
    threading.Thread(target=reader,daemon=True).start()
    follow=None;observations=Observations();last_new=time.monotonic();last_beat=0
    print('Startup STOP. Commands: arm | goal X Y | follow CLASS COLOR | stop | quit')
    print('Sent is NOT an acknowledgement; prototype firmware does not yet return task ACKs.')
    link.stop()
    try:
        while True:
            now=time.monotonic()
            try:cmd=commands.get_nowait().split()
            except queue.Empty:cmd=[]
            if cmd:
                try:
                    if cmd==['quit']:break
                    if cmd==['stop']:link.stop();follow=None
                    elif cmd==['arm']:link.arm();follow=None
                    elif len(cmd)==3 and cmd[0]=='goal':link.map_goal(float(cmd[1]),float(cmd[2]));follow=None
                    elif len(cmd)==3 and cmd[0]=='follow':
                        if not args.camera_calibrated:raise ValueError('Calibrate camera/range before enabling follow')
                        follow=(int(cmd[1]),int(cmd[2]));observations=Observations();last_new=now;link.follow()
                    else:print('Unknown command')
                except (ValueError,OverflowError) as e:print(e)
            if follow is not None:
                try:
                    data=json.loads(Path(args.observations).read_text())
                    obj=observations.select(data,time.monotonic()*1000,*follow)
                except (OSError,ValueError,TypeError):obj=None
                if obj:
                    link.target(obj['bearing_rad'],int(obj['range_mm']));last_new=now
                elif now-last_new>.18:
                    link.stop();follow=None;print('Target lost/stale: stopped; new explicit task required.')
            if now-last_beat>.1:link.heartbeat();last_beat=now
            if serial and serial.in_waiting:
                # Drain telemetry so host TX flow cannot fill; no fabricated ACK.
                serial.read(serial.in_waiting)
            time.sleep(.02)
    finally:
        try:link.stop()
        finally:
            if serial:serial.close()
            if lock:lock.close()

if __name__=='__main__':main()
