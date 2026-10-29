#!/usr/bin/env python3
"""Complete a registered normal cohort without retrying or discarding failed seeds."""
import argparse
import hashlib
import json
import math
import os
import random
import subprocess
from pathlib import Path
import yaml


def inspect(folder,start=42,count=30,factor=1.,complete=True):
    records={}; issues=[]; versions=set(); middlewares=set(); failed=[]; unverified_stop=[]
    for path in sorted(Path(folder).rglob('metrics.json')):
        sample=json.loads(path.read_text()); seed=sample['seed']
        if seed in records: issues.append(f'duplicate seed {seed}')
        records[seed]=(sample,path)
        if sample.get('scenario')!='happy' or sample.get('profile')!='baseline': issues.append(f'{path}: wrong scenario/profile')
        if not start<=seed<start+count: issues.append(f'{path}: seed outside registered cohort')
        manifest=yaml.safe_load(path.with_name('manifest.yaml').read_text())
        middlewares.add(manifest.get('rmw'))
        if not manifest.get('headless') or manifest.get('target_real_time_factor')!=factor: issues.append(f'{path}: environment mismatch')
        hashes=manifest.get('source_hashes',{})
        if not hashes: issues.append(f'{path}: missing source hashes')
        else: versions.add(tuple(sorted(hashes.items())))
        if sample.get('collision_count')!=0: issues.append(f'{path}: collision not verified zero')
        if sample.get('passed'):
            if not (sample.get('success') and sample.get('truth_valid') and sample.get('physical_stop_verified') and
                    sample.get('final_state')==9 and sample.get('load_state')==0 and not sample.get('requires_operator')):
                issues.append(f'{path}: invalid successful terminal evidence')
        else:
            failed.append({'seed':seed,'path':str(path),'error_code':sample.get('error_code'),'error':sample.get('error')})
            if not sample.get('physical_stop_verified'): unverified_stop.append(seed)
            if not (sample.get('accepted') and not sample.get('success') and sample.get('final_state')==10 and sample.get('requires_operator')):
                issues.append(f'{path}: failure is not a bounded FAILED/HOLD outcome')
            truth=path.with_name('truth.jsonl')
            samples=[json.loads(line) for line in truth.read_text().splitlines()] if truth.exists() else []
            if not samples or any(s['collision_count']!=0 or s['collision'] or not all(math.isfinite(s[k]) for k in ('x','y','yaw','source_sim_s')) for s in samples):
                issues.append(f'{path}: failed sample lacks valid zero-contact raw trace')
            elif any(not 0<b['source_sim_s']-a['source_sim_s']<=.5 for a,b in zip(samples,samples[1:])):
                issues.append(f'{path}: failed sample truth has missing/nonmonotonic intervals')
            events=list(path.parent.glob('*/events.jsonl'))
            terminal=[]
            for event_file in events:
                terminal.extend(e for e in map(json.loads,event_file.read_text().splitlines()) if e['event_type']=='terminal')
            if len(terminal)!=1 or not samples or samples[-1]['source_sim_s']<terminal[0]['ros_stamp']-.1:
                issues.append(f'{path}: failed sample lacks truth covering bounded terminal')
    if len(versions)>1: issues.append('mixed source versions in normal cohort')
    if None in middlewares or len(middlewares)>1: issues.append('missing or mixed middleware in normal cohort')
    if complete and set(records)!=set(range(start,start+count)): issues.append('registered normal seeds are incomplete')
    passed=sum(bool(sample['passed']) for sample,_ in records.values())
    summary={'n':len(records),'passed':passed,'success_rate':passed/len(records) if records else None,
             'required_success_rate':.95,'registered_seeds':list(range(start,start+count)),
             'source_versions':len(versions),'failed_samples':failed,'physical_stop_unverified_seeds':unverified_stop,
             'middleware':sorted(m for m in middlewares if m),
             'issues':issues,'meets_target':len(records)==count and passed/count>=.95 and not issues}
    return records,summary


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',required=True)
    parser.add_argument('--seed',type=int,default=42); parser.add_argument('--count',type=int,default=30)
    parser.add_argument('--real-time-factor',type=float,default=1.); parser.add_argument('--audit-only',action='store_true')
    args=parser.parse_args(); folder=Path(args.output).resolve(); folder.mkdir(parents=True,exist_ok=True)
    records,summary=inspect(folder,args.seed,args.count,args.real_time_factor,complete=False)
    if summary['issues']: raise ValueError(summary['issues'])
    selected=os.environ.get('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    if summary['middleware'] and summary['middleware']!=[selected]:
        raise ValueError('existing cohort middleware differs from current environment; use a new output directory')
    if records and not args.audit_only:
        from ament_index_python.packages import get_package_share_directory as share
        current={}
        for package in ('amr_description','amr_simulation','amr_navigation','amr_mission','amr_bringup','amr_interfaces'):
            source=Path(share(package)).parents[3]/'src'/package
            for file in source.rglob('*'):
                if file.is_file() and '__pycache__' not in file.parts and file.suffix!='.pyc':
                    current[f'{package}/{file.relative_to(source)}']=hashlib.sha256(file.read_bytes()).hexdigest()
        expected=yaml.safe_load(next(iter(records.values()))[1].with_name('manifest.yaml').read_text())['source_hashes']
        if current!=expected: raise ValueError('installed source differs from existing cohort; use a new output directory')
    rng=random.Random(args.seed); domains={seed:rng.randrange(30,90) for seed in range(args.seed,args.seed+args.count)}
    if not args.audit_only:
        for seed,domain in domains.items():
            if seed in records: continue  # A failed seed is an observation, never a retry slot.
            print(f'FILL registered normal seed={seed} domain={domain}',flush=True)
            subprocess.run(['ros2','run','amr_bringup','scenario_runner','--cases','happy','--repeat','1',
                '--seed',str(seed),'--domain-id',str(domain),'--headless','--real-time-factor',str(args.real_time_factor),'--output',str(folder)],check=False)
            records,summary=inspect(folder,args.seed,args.count,args.real_time_factor,complete=False)
            if summary['issues'] or len(records)-summary['passed']>args.count*.05: break
    records,summary=inspect(folder,args.seed,args.count,args.real_time_factor)
    (folder/'normal_sampling_audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    return 0 if summary['meets_target'] else 1


if __name__=='__main__': raise SystemExit(main())
