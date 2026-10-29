#!/usr/bin/env python3
"""Render paired raw M4 measurements, retaining failures in the exported table."""
import argparse
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True,help='Output prefix, without extension')
    args=parser.parse_args()
    records=[]; keys=set()
    for path in Path(args.input).rglob('metrics.json'):
        sample=json.loads(path.read_text())
        if sample.get('scenario')!='narrow': continue
        key=(sample['profile'],sample['seed'])
        if key in keys: raise ValueError('Duplicate profile/seed; choose one formal experiment directory')
        keys.add(key); records.append(sample)
    if not records: raise ValueError('No narrow-aisle measurements')
    prefix=Path(args.output); prefix.parent.mkdir(parents=True,exist_ok=True)
    columns=['profile','effective_profile','seed','accepted','passed','success','collision_count','truth_valid','physical_stop_verified','route',
             'elapsed_sim_s','elapsed_wall_s','path_length_m','minimum_clearance_m','controller_cpu_s','observed_real_time_factor','error']
    with prefix.with_suffix('.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore'); writer.writeheader()
        writer.writerows(sorted(records,key=lambda s:(s['profile'],s['seed'])))
    figure,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for profile,color in [('baseline','#5164a1'),('candidate','#16817a')]:
        samples=sorted((s for s in records if s['profile']==profile),key=lambda s:s['seed'])
        if not samples: continue
        for axis,clock in zip(axes,['sim','wall']):
            key='elapsed_'+clock+'_s'
            valid=[s for s in samples if isinstance(s.get(key),(int,float)) and math.isfinite(s[key])]
            passed=[s for s in valid if s['passed']]
            failed=[s for s in valid if not s['passed']]
            axis.plot([s['seed'] for s in passed],[s[key] for s in passed],'.-',color=color,linewidth=1,
                      label=f'{profile}: {len(passed)}/{len(samples)} passed')
            if failed: axis.scatter([s['seed'] for s in failed],[s[key] for s in failed],marker='x',color=color,s=60)
            values=sorted(s[key] for s in passed)
            if values: axis.axhline(values[math.ceil(.95*len(values))-1],color=color,linestyle='--',alpha=.5)
            axis.set(xlabel='Paired seed',ylabel=f'{clock} seconds',title=f'Goal duration ({clock} clock)')
            axis.grid(alpha=.2); axis.legend(fontsize=8)
    figure.suptitle('AMR narrow-aisle experiment | dashed: nearest-rank P95 | x: failed assertion')
    figure.savefig(prefix.with_suffix('.png'),dpi=160)
    plt.close(figure)
    print(prefix.with_suffix('.png'))


if __name__=='__main__': main()
