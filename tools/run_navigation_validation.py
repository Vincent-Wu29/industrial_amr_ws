#!/usr/bin/env python3
"""M1: 15 real goals, controlled SLAM/map save, then three saved-map goals."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

def stop(process):
    if process is None: return
    for sig,timeout in [(signal.SIGINT,12),(signal.SIGTERM,4),(signal.SIGKILL,2)]:
        try: os.killpg(process.pid,sig)
        except ProcessLookupError: pass
        try: process.wait(timeout=timeout)
        except subprocess.TimeoutExpired: continue
    process.wait()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default='artifacts/m1_validation'); p.add_argument('--repeat',type=int,default=5); p.add_argument('--skip-mapping',action='store_true'); p.add_argument('--mapping-only',action='store_true')
    p.add_argument('--real-time-factor',type=float,default=1.)
    args=p.parse_args(); root=Path(args.output).resolve(); root.mkdir(parents=True,exist_ok=True)
    summary={}; process=None; system_log=None; env=os.environ.copy()
    def launch(mode,folder,extra=None):
        nonlocal process,system_log,env
        folder.mkdir(parents=True,exist_ok=True); env=os.environ.copy()
        env['ROS_DOMAIN_ID']=str(40+int(uuid.uuid4().hex[:4],16)%50); env['GZ_PARTITION']='amr_'+uuid.uuid4().hex
        system_log=(folder/'system.log').open('w')
        process=subprocess.Popen(['ros2','launch','amr_bringup','system.launch.py',f'mode:={mode}',
            'gui:=false',f'artifacts_dir:={folder}',f'real_time_factor:={args.real_time_factor}']+(extra or []),env=env,stdout=system_log,stderr=subprocess.STDOUT,start_new_session=True)
    def close():
        nonlocal process,system_log
        stop(process); process=None
        if system_log: system_log.close(); system_log=None
    def command(cmd,log,timeout):
        print('RUN '+' '.join(cmd),flush=True)
        with log.open('w') as output:
            child=subprocess.Popen(cmd,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                end=time.monotonic()+timeout
                while child.poll() is None:
                    if process.poll() is not None: raise RuntimeError('system exited; '+str(log))
                    if time.monotonic()>end: raise TimeoutError(str(cmd))
                    time.sleep(.2)
                if child.returncode: raise RuntimeError(f'command failed ({child.returncode}); {log}')
            finally: stop(child)
    def check_truth(folder):
        path=folder/'truth.jsonl'
        samples=[json.loads(line) for line in path.read_text().splitlines()]
        if not samples: raise RuntimeError('missing physical truth')
        collisions=max(s['collision_count'] for s in samples)
        if collisions: raise RuntimeError(f'{collisions} geometry collisions; {folder}')
        return {'samples':len(samples),'collision_count':collisions}
    try:
        if not args.mapping_only:
            folder=root/'fixed_map'; launch('navigation',folder)
            command(['ros2','run','amr_bringup','readiness_check','--timeout','120'],folder/'readiness.log',140)
            command(['ros2','run','amr_navigation','navigation_smoke','--repeat',str(args.repeat),'--output',str(folder/'navigation_smoke.json')],folder/'goals.log',args.repeat*1000)
            summary['fixed_map']=check_truth(folder); summary['fixed_map']['goals']=json.loads((folder/'navigation_smoke.json').read_text()); close()
        if not args.skip_mapping:
            folder=root/'mapping'; launch('mapping',folder); map_prefix=folder/'mapped_warehouse_v1'
            command(['ros2','run','amr_navigation','mapping_drive','--save',str(map_prefix)],folder/'mapping.log',2000)
            summary['mapping']=check_truth(folder); close()
            folder=root/'saved_map'; launch('navigation',folder,[f'map:={map_prefix}.yaml',f'stations:={map_prefix}_stations.yaml'])
            command(['ros2','run','amr_bringup','readiness_check','--timeout','120'],folder/'readiness.log',140)
            command(['ros2','run','amr_navigation','navigation_smoke','--repeat','1','--stations-file',str(map_prefix)+'_stations.yaml','--output',str(folder/'navigation_smoke.json')],folder/'goals.log',1000)
            summary['saved_map']=check_truth(folder); summary['saved_map']['goals']=json.loads((folder/'navigation_smoke.json').read_text()); close()
        summary['passed']=True; return 0
    except Exception as e:
        summary.update(passed=False,error=f'{type(e).__name__}: {e}'); print(summary['error'],flush=True); return 1
    finally:
        close(); (root/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8'); print(json.dumps(summary),flush=True)
if __name__=='__main__': raise SystemExit(main())
