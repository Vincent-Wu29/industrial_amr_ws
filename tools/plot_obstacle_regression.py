#!/usr/bin/env python3
"""Plot actual ECM traces for a failed fixture case and its repair regression."""
import argparse
import ast
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle


def corners(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+a*c-b*s,y+a*s+b*c) for a,b in [(.2,.15),(.2,-.15),(-.2,-.15),(-.2,.15)]]


def point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def trace(folder,intersects):
    read=lambda name:[json.loads(line) for line in (folder/name).read_text().splitlines() if line.strip()]
    robot=read('truth.jsonl'); boxes=[v for v in read('obstacle_truth.jsonl') if v['fixture']=='scenario_box']
    times=[v['sim_s'] for v in boxes]; samples=[]
    for sample in robot:
        stamp=sample.get('source_sim_s',sample['sim_s']); index=bisect_right(times,stamp+1e-6)-1
        if index<0: continue
        box=boxes[index]; bx,by=box['x'],box['y']
        if not 0<bx<20 or not 0<by<15: continue
        polygon=corners(sample['x'],sample['y'],sample['yaw'])
        obstacle=[(bx+a,by+b) for a,b in [(.3,.3),(.3,-.3),(-.3,-.3),(-.3,.3)]]
        if intersects(sample['x'],sample['y'],sample['yaw'],(bx,by,.6,.6)): gap=0.
        else:
            gap=min(point_segment(p,other[i],other[(i+1)%4])
                    for shape,other in [(polygon,obstacle),(obstacle,polygon)] for p in shape for i in range(4))
        samples.append({'sim_s':stamp,'x':sample['x'],'y':sample['y'],'yaw':sample['yaw'],
                        'box_x':bx,'box_y':by,'gap_m':gap,'collision':sample['collision']})
    if not samples: raise ValueError('no actual active fixture samples in '+str(folder))
    return samples


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--before',required=True)
    parser.add_argument('--after',required=True); parser.add_argument('--output',required=True)
    args=parser.parse_args(); root=Path(__file__).resolve().parents[1]
    source=ast.parse((root/'src/amr_simulation/scripts/truth_monitor').read_text(encoding='utf-8'))
    function=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='intersects')
    namespace={'math':math}; exec(compile(ast.Module(body=[function],type_ignores=[]),'geometry','exec'),namespace)
    records=[trace(Path(folder),namespace['intersects']) for folder in (args.before,args.after)]
    prefix=Path(args.output); prefix.parent.mkdir(parents=True,exist_ok=True)
    with prefix.with_suffix('.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=['version']+list(records[0][0]))
        writer.writeheader()
        for version,samples in zip(['before','after'],records):
            writer.writerows({'version':version,**sample} for sample in samples)
    fig,axes=plt.subplots(1,2,figsize=(11,5),layout='constrained')
    xvalues=[s['x'] for group in records for s in group]+[s['box_x'] for group in records for s in group]
    yvalues=[s['y'] for group in records for s in group]+[s['box_y'] for group in records for s in group]
    for axis,label,samples in zip(axes,['Before monitor','With monitor'],records):
        focus=next((s for s in samples if s['collision']),min(samples,key=lambda s:s['gap_m']))
        axis.plot([s['x'] for s in samples],[s['y'] for s in samples],color='#355e8c',label='AMR actual path')
        axis.plot([s['box_x'] for s in samples],[s['box_y'] for s in samples],'--',color='#cb7b36',label='Box actual path')
        axis.add_patch(Polygon(corners(focus['x'],focus['y'],focus['yaw']),facecolor='#355e8c',alpha=.6,label='AMR at closest approach'))
        axis.add_patch(Rectangle((focus['box_x']-.3,focus['box_y']-.3),.6,.6,facecolor='#cb7b36',alpha=.6,label='Box at same source time'))
        contact=[s for s in samples if s['collision']]
        if contact: axis.scatter([s['x'] for s in contact],[s['y'] for s in contact],s=18,color='#b83b43',label='Recorded geometric contact')
        axis.set(xlabel='World x / m',ylabel='World y / m',title=f'{label}: minimum gap {min(s["gap_m"] for s in samples):.3f} m',
                 xlim=(min(xvalues)-.45,max(xvalues)+.45),ylim=(min(yvalues)-.45,max(yvalues)+.45))
        axis.set_aspect('equal'); axis.grid(alpha=.2); axis.legend(fontsize=7,loc='lower right')
    fig.suptitle('Crossing fixture, seed 46 | actual ECM geometry, single-case repair evidence')
    fig.savefig(prefix.with_suffix('.png'),dpi=160); plt.close(fig)
    print(prefix.with_suffix('.png'))


if __name__=='__main__': main()
