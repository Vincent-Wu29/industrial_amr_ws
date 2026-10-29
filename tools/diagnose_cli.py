#!/usr/bin/env python3
"""Preserve each CLI/fake-Mission diagnostic attempt, including early terminal races."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--repeat',type=int,default=5)
    args=parser.parse_args()
    if not 1<=args.repeat<=10: parser.error('repeat must be 1..10')
    folder=Path(args.output).resolve(); folder.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1]
    source=root/'src/amr_mission/test/test_ros_integration.py'
    spec=importlib.util.spec_from_file_location('cli_fixture',source)
    fixture_module=importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture_module)
    original=subprocess.Popen; attempts=[]
    try:
        for index in range(args.repeat):
            captured=[]
            class Capture(original):
                def communicate(self,*a,**kw):
                    streams=super().communicate(*a,**kw)
                    captured.append({'returncode':self.returncode,'stdout':streams[0],'stderr':streams[1]})
                    return streams
            subprocess.Popen=Capture
            run=folder/f'{index:02d}'; run.mkdir()
            began=time.monotonic(); fixture=fixture_module.system.__wrapped__(run); node=None
            record={'index':index,'passed':False,'error':'','cli':captured}
            try:
                node=next(fixture)
                fixture_module.test_public_cli_bounds_missing_state_channel(node)
                record['passed']=True
            except Exception as error:
                record['error']=repr(error)
            finally:
                if node and node.state:
                    record['last_state']={key:getattr(node.state,key) for key in ('state','load_state','error_code','run_id','task_id')}
                fixture.close()
                record['elapsed_wall_s']=time.monotonic()-began
                attempts.append(record)
                (run/'cli_capture.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')
                print(json.dumps(record,ensure_ascii=False),flush=True)
    finally:
        subprocess.Popen=original
    result={'attempts':attempts,'all_passed':all(r['passed'] for r in attempts),'source_hashes':{
        str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [source,root/'src/amr_mission/scripts/task_cli']}}
    (folder/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if result['all_passed'] else 1


if __name__=='__main__': raise SystemExit(main())
