#!/usr/bin/env python3
"""Show recorded diagnostic costmaps/plans; never use these examples as an A/B cohort."""
import argparse
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
import numpy as np
import yaml
from analyze_navigation_probe import analyze,load


def snapshot(folder,blocked):
    summary=analyze(folder); records=summary['timed_plan_costs']
    if not records: raise ValueError('timed native plan required')
    selected=next((r for r in records if blocked and r['lethal_columns']>0),records[0])
    cost=load(folder,'global_costmaps')[selected['costmap_index']]
    plan=next(p for p in load(folder,'plans') if p['sim_s']==selected['sim_s'])
    truth=load(folder,'truth')
    actual=min(truth,key=lambda p:abs(p['source_sim_s']-cost['sim_s']))
    config=yaml.safe_load((folder/'nav2.yaml').read_text())
    marking=config['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['obstacle_max_range']
    return selected,cost,plan,actual,marking


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--before',required=True,help='One recorded failing diagnostic run')
    parser.add_argument('--after',required=True,help='One near-field diagnostic run')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    colors=plt.get_cmap('YlOrBr')(np.linspace(0,1,256))
    colors[253]=[.82,.2,.2,1]; colors[254]=[.10,.12,.15,1]; colors[255]=[.7,.7,.7,1]
    cmap=ListedColormap(colors)
    figure,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for axis,folder,blocked in zip(axes,[Path(args.before),Path(args.after)],[True,False]):
        record,cost,plan,robot,marking=snapshot(folder,blocked)
        matrix=np.asarray(cost['rows'],dtype=np.uint8); r=cost['resolution']
        extent=[cost['x0'],cost['x0']+matrix.shape[1]*r,cost['y0'],cost['y0']+matrix.shape[0]*r]
        heat=axis.imshow(matrix,origin='lower',extent=extent,vmin=0,vmax=255,cmap=cmap,interpolation='nearest')
        axis.plot([p[0] for p in plan['points']],[p[1] for p in plan['points']],color='#16817a',linewidth=2,label='Native global plan')
        for y in [12.,12.8]: axis.plot([8,12],[y,y],color='#3185bf',linestyle='--',linewidth=1)
        rectangle=Rectangle((robot['x']-.2,robot['y']-.15),.4,.3,facecolor='none',edgecolor='#222222',linewidth=1.2,label='Actual AMR body')
        rectangle.set_transform(Affine2D().rotate_around(robot['x'],robot['y'],robot['yaw'])+axis.transData); axis.add_patch(rectangle)
        ghosts=[(cost['x0']+(i+.5)*r,cost['y0']+(j+.5)*r) for j,row in enumerate(matrix) for i,v in enumerate(row)
                if v==254 and 8.5<=cost['x0']+(i+.5)*r<=11.5 and 12.15<=cost['y0']+(j+.5)*r<=12.65]
        if ghosts: axis.scatter(*zip(*ghosts),marker='x',color='#db2244',s=35,label='Lethal in empty interior',zorder=5)
        axis.set(xlim=(6.5,13.5),ylim=(11,14.6),xlabel='Map x / m',ylabel='Map y / m',
                 title=f"Global marking {marking:g} m | blocked columns {record['lethal_columns']}\nCostmap t={cost['sim_s']:.2f}, plan t={plan['sim_s']:.2f} sim s")
        axis.set_aspect('equal'); axis.legend(loc='upper left',fontsize=7)
    bar=figure.colorbar(heat,ax=list(axes),shrink=.65,ticks=[0,80,160,253]); bar.set_ticklabels(['0 free','80','160','253+ blocked'])
    figure.suptitle('Recorded native costmaps and plans | first blocked / first goal snapshot | diagnostic examples')
    destination=Path(args.output); destination.parent.mkdir(parents=True,exist_ok=True)
    figure.savefig(destination,dpi=170); plt.close(figure); print(destination)


if __name__=='__main__': main()
