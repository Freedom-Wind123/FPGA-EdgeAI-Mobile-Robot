from pathlib import Path
import subprocess,sys,json,xml.etree.ElementTree as ET,binascii,struct,hashlib
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'evidence';OUT.mkdir(exist_ok=True)
IV='F:/iverilog/bin/iverilog.exe';VVP='F:/iverilog/bin/vvp.exe'
def frame(k,seq,ver=1):
 b=struct.pack('<BBH',ver,k,seq)+bytes(24)
 return b'\xaa\x55'+b+struct.pack('<H',binascii.crc_hqx(b,0xffff))
bad=bytearray(frame(3,2));bad[12]^=8
frames=frame(3,1)+bad+frame(0x10,2)+frame(2,0)+frame(0x11,3)+frame(3,4,2)
(OUT/'frames.hex').write_text('\n'.join(f'{b:02x}' for b in frames))
sources=[str(p) for p in (ROOT/'fpga/rtl').glob('*.sv')]
baseline=[str(p) for p in (ROOT/'baseline/chassis/rtl').glob('*.sv') if p.name not in ('chassis_top.sv','telemetry_formatter.sv')]
results=[]
for name in ['guard','lidar','protocol','mission','map','bfs','servo','projector','sectors','core','telemetry','telemetry_equivalence']:
 tb=ROOT/f'fpga/tests/tb_{name}.sv'
 if not tb.exists():continue
 extra=[str(ROOT/'fpga/tests/telemetry_formatter_ref.sv')] if name=='telemetry_equivalence' else []
 cmd=[IV,'-g2012','-I',str(ROOT/'baseline/chassis/rtl'),'-s','tb_'+name,'-o',str(OUT/(name+'.vvp')),*sources,*baseline,str(tb),*extra]
 p=subprocess.run(cmd,capture_output=True,text=True)
 log=p.stdout+p.stderr
 if p.returncode==0:
  p=subprocess.run([VVP,str(OUT/(name+'.vvp'))],cwd=ROOT,capture_output=True,text=True,timeout=60)
  log+=p.stdout+p.stderr
 (OUT/(name+'.log')).write_text(log)
 results.append({'test':name,'pass':p.returncode==0})
 print(name,'PASS' if p.returncode==0 else 'FAIL\n'+log[-2400:],flush=True)
# Elaborate complete board, replacing only vendor IPs with existing simulation stubs.
project=ET.parse(ROOT/'fpga/robot_board_top.gprj')
exclude=('camera_pll/','ddr_pll/','ddr3_memory_interface/','rd_data_fifo/',
         'wr_data_fifo/','gowin_ddr/','src/pll_init.v','src/pll_mDRP_intf.v')
src=[str((ROOT/'fpga'/e.attrib['path']).resolve()) for e in project.findall('.//File')
 if e.attrib['type']=='file.verilog' and not any(x in e.attrib['path'] for x in exclude)]
stub=ROOT/'reference/ip_stubs.sv'
p=subprocess.run([IV,'-g2012','-Wall','-I',str(ROOT/'baseline/chassis/rtl'),'-I',str(ROOT/'baseline/src/vga'),
 '-s','robot_board_top','-o',str(OUT/'board.vvp'),*src,str(stub)],capture_output=True,text=True)
(OUT/'board_elaboration.log').write_text(p.stdout+p.stderr)
results.append({'test':'board_elaboration_with_IP_stubs','pass':p.returncode==0})
print('board elaboration',p.returncode,flush=True)
if p.returncode:print((p.stdout+p.stderr)[-2500:])
if (ROOT/'rk3588/test_link.py').exists():
 p=subprocess.run([sys.executable,'-m','unittest','test_link','-v'],cwd=ROOT/'rk3588',capture_output=True,text=True)
 (OUT/'python_tests.log').write_text(p.stdout+p.stderr)
 results.append({'test':'python_protocol_9_cases','pass':p.returncode==0})
 print(p.stdout+p.stderr)
(OUT/'tests.json').write_text(json.dumps(results,indent=2))
(OUT/'test_sources.json').write_text(json.dumps({p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
 for d in ['fpga','baseline','rk3588'] for p in (ROOT/d).rglob('*') if p.is_file() and p.suffix in ('.sv','.svh','.v','.py')},indent=2))
sys.exit(0 if all(r['pass'] for r in results) else 1)
