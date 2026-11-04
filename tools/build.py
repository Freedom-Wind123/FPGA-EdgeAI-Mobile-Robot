"""Isolated Gowin synthesis. NO programming; resource-check includes motion logic.

Release synthesis defaults to hardware lock. --resource-check is ONLY for resource
estimation, never SRAM/Flash. Timing closure and hardware tests remain mandatory.
"""
from pathlib import Path
import argparse,datetime,hashlib,json,shutil,subprocess,re,html
ROOT=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('--resource-check',action='store_true')
ap.add_argument('--pnr',action='store_true',help='Also run place/route for diagnostics; never programs hardware')
ap.add_argument('--stage',required=True)
args=ap.parse_args();stage=Path(args.stage).resolve()
allowed=Path('F:/27_qiansai/build_runs').resolve()
if allowed not in stage.parents or not str(stage).isascii():raise SystemExit('New ASCII path under build_runs required')
stage.mkdir(exist_ok=False)
for d in ['baseline','fpga']:
 shutil.copytree(ROOT/d,stage/d,ignore=shutil.ignore_patterns('impl','*.vvp','*.bak','*.user'))
if args.resource_check:
 p=stage/'fpga/rtl/robot_board_top.sv';s=p.read_text();s=s.replace('parameter HARDWARE_CONFIRMED=0','parameter HARDWARE_CONFIRMED=1');p.write_text(s)
if args.pnr:
 p=stage/'fpga/synth.tcl';p.write_text(p.read_text().replace('run syn','run all'))
manifest={p.relative_to(stage).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in stage.rglob('*') if p.is_file()}
(stage/'build_inputs.json').write_text(json.dumps({'resource_only':args.resource_check,'files':manifest},indent=2))
with (stage/'gowin_synthesis.log').open('w',encoding='utf-8') as log:
 p=subprocess.run(['F:/Gowin/Gowin_V1.9.12_x64/IDE/bin/gw_sh.exe','synth.tcl'],cwd=stage/'fpga',stdout=log,stderr=subprocess.STDOUT)
result={'tool_exit_code':p.returncode,'synthesis_only':not args.pnr,'resource_only':args.resource_check,
        'timing_closed':False,'board_tested':False,'release_approved':False}
exit_code=p.returncode
if args.pnr and p.returncode==0:
 report=stage/'fpga/impl/pnr/robot_board_top_tr_content.html'
 text=html.unescape(re.sub('<[^>]+>',' ',report.read_text(errors='replace'))) if report.exists() else ''
 text=re.sub(r'\s+',' ',text)
 for key,label in [('setup','Setup'),('hold','Hold')]:
  m=re.search('Numbers of '+label+r' Violated Endpoints\s+(\d+)',text)
  result[key+'_violated_endpoints']=int(m.group(1)) if m else None
 # Even zero setup/hold would still require CDC, recovery/removal and board review.
 result['setup_hold_zero']=result['setup_violated_endpoints']==0 and result['hold_violated_endpoints']==0
 if not result['setup_hold_zero']:exit_code=2
(stage/'result.json').write_text(json.dumps(result,indent=2))
print(stage,exit_code,flush=True)
raise SystemExit(exit_code)
