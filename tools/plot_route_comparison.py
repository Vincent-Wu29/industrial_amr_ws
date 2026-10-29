#!/usr/bin/env python3
"""Plot one registered seed's unfiltered Gazebo traces and world geometry."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--seed',type=int,default=42)
    args=parser.parse_args(); records={}
    for path in Path(args.input).rglob('metrics.json'):
        sample=json.loads(path.read_text())
        if sample.get('scenario')=='narrow' and sample.get('seed')==args.seed:
            profile=sample['profile']
            if profile in records: raise ValueError('duplicate profile/seed')
            records[profile]=(sample,path.parent)
    if set(records)!={'baseline','candidate'}: raise ValueError('paired seed required')
    world=ET.parse(records['baseline'][1]/'effective_world.sdf').getroot()
    boxes=[]
    for model in world.findall('.//world/model'):
        if model.get('name')=='floor': continue
        pose=[float(n) for n in model.findtext('pose','0 0 0 0 0 0').split()]
        for box in model.findall('link/collision/geometry/box'):
            size=[float(n) for n in box.findtext('size').split()]
            boxes.append((pose[0],pose[1],size[0],size[1],pose[5]))
    figure,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for axis in axes:
        for x,y,w,h,yaw in boxes:
            rect=Rectangle((x-w/2,y-h/2),w,h,facecolor='#d5d9de',edgecolor='#9098a3',linewidth=.6)
            rect.set_transform(Affine2D().rotate_around(x,y,yaw)+axis.transData); axis.add_patch(rect)
        axis.set_aspect('equal'); axis.grid(alpha=.15); axis.set(xlabel='Gazebo x / m',ylabel='Gazebo y / m')
    colors={'baseline':'#5164a1','candidate':'#16817a'}
    for profile,(sample,folder) in records.items():
        points=[json.loads(line) for line in (folder/'truth.jsonl').read_text().splitlines()]
        for axis in axes:
            axis.plot([p['x'] for p in points],[p['y'] for p in points],color=colors[profile],linewidth=1.8,
                      label=f"{profile}: {sample['route']}, {sample['elapsed_sim_s']:.3f} sim s")
    axes[0].set(xlim=(0,20),ylim=(0,15),title='Whole run: staging and measured navigation')
    axes[1].set(xlim=(6.6,13.4),ylim=(11.0,14.6),title='Narrow passage and upper bypass (same traces)')
    for axis in axes:
        axis.scatter([7.,13.],[12.4,12.4],marker='x',s=50,color='#222222',zorder=4)
        axis.legend(loc='lower right',fontsize=7)
    figure.suptitle(f'Gazebo geometry and raw ground-truth trajectories | paired seed {args.seed}')
    destination=Path(args.output); destination.parent.mkdir(parents=True,exist_ok=True)
    figure.savefig(destination,dpi=170); plt.close(figure); print(destination)


if __name__=='__main__': main()
