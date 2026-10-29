#!/usr/bin/env python3
"""Instrument unmodified narrow workers; diagnostic runs never enter formal cohorts."""
import argparse
import hashlib
import importlib.machinery
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid


def load_runner():
    root=Path(__file__).resolve().parents[1]
    path=root/'src/amr_bringup/scripts/scenario_runner'
    loader=importlib.machinery.SourceFileLoader('observed_scenario',str(path))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    runner=importlib.util.module_from_spec(spec); loader.exec_module(runner)
    return runner


def observed_worker(args):
    import rclpy.node
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Path as NavPath
    from nav2_msgs.msg import Costmap
    folder=Path(args.output).resolve(); folder.mkdir(parents=True,exist_ok=True)
    streams={name:(folder/(name+'.jsonl')).open('w') for name in ('plans','global_costmaps','amcl')}
    def record(name,value):
        streams[name].write(json.dumps(value)+'\n'); streams[name].flush()
    stamp=lambda header:header.stamp.sec+header.stamp.nanosec*1e-9
    original=rclpy.node.Node
    class ObservedNode(original):
        def __init__(self,*a,**kw):
            super().__init__(*a,**kw)
            def plan(message):
                record('plans',{'sim_s':stamp(message.header),'frame':message.header.frame_id,
                    'points':[[p.pose.position.x,p.pose.position.y] for p in message.poses]})
            def pose(message):
                p=message.pose.pose; q=p.orientation
                record('amcl',{'sim_s':stamp(message.header),'x':p.position.x,'y':p.position.y,
                    'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                    'covariance':[message.pose.covariance[i] for i in (0,7,35)]})
            def costmap(message):
                m=message.metadata; resolution=m.resolution
                x0=m.origin.position.x; y0=m.origin.position.y
                i0=max(0,int((6.5-x0)/resolution)); i1=min(m.size_x,int((13.5-x0)/resolution)+1)
                j0=max(0,int((11.-y0)/resolution)); j1=min(m.size_y,int((14.6-y0)/resolution)+1)
                rows=[list(message.data[j*m.size_x+i0:j*m.size_x+i1]) for j in range(j0,j1)]
                record('global_costmaps',{'sim_s':stamp(message.header),'frame':message.header.frame_id,
                    'resolution':resolution,'x0':x0+i0*resolution,'y0':y0+j0*resolution,'rows':rows})
            self.create_subscription(NavPath,'/plan',plan,10)
            self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose,10)
            self.create_subscription(Costmap,'/global_costmap/costmap_raw',costmap,1)
    rclpy.node.Node=ObservedNode
    (folder/'probe_manifest.json').write_text(json.dumps({'diagnostic_only':True,'global_range_override':args.global_range,
        'instrumentation_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2))
    try:
        runner=load_runner()
        if args.global_range is not None:
            def observed_overlay(base,changes):
                def merge(target,fragment):
                    for key,value in fragment.items():
                        if isinstance(value,dict): merge(target.setdefault(key,{}),value)
                        else: target[key]=value
                merge(base,changes)
                base['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['obstacle_max_range']=args.global_range
            runner.overlay=observed_overlay
        return runner.worker(argparse.Namespace(output=str(folder),case='narrow',profile=args.profile,
            headless=True,seed=args.seed,real_time_factor=args.real_time_factor))
    finally:
        rclpy.node.Node=original
        for stream in streams.values(): stream.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--profile',choices=['baseline','candidate'],default='candidate')
    parser.add_argument('--seed',type=int,default=45)
    parser.add_argument('--repeat',type=int,default=3)
    parser.add_argument('--real-time-factor',type=float,default=3.)
    parser.add_argument('--global-range',type=float)
    parser.add_argument('--worker',action='store_true')
    args=parser.parse_args()
    if args.worker: return observed_worker(args)
    if not 1<=args.repeat<=5: parser.error('repeat must be 1..5')
    root=Path(args.output).resolve(); root.mkdir(parents=True,exist_ok=False)
    runner=load_runner(); results=[]
    for index in range(args.repeat):
        folder=root/f'{index:02d}'; env=os.environ.copy()
        env.update(ROS_DOMAIN_ID=str(80+index),GZ_PARTITION='amr_probe_'+uuid.uuid4().hex)
        command=[sys.executable,__file__,'--worker','--output',str(folder),'--profile',args.profile,
                 '--seed',str(args.seed),'--real-time-factor',str(args.real_time_factor)]
        if args.global_range is not None: command+=['--global-range',str(args.global_range)]
        child=subprocess.Popen(command,env=env,start_new_session=True)
        try: child.wait(timeout=1500)
        finally:
            if child.poll() is None: runner.stop(child)
        result=json.loads((folder/'metrics.json').read_text()); results.append(result)
    (root/'probe_summary.json').write_text(json.dumps(results,indent=2))
    return 0 if all(r['passed'] for r in results) else 1


if __name__=='__main__': raise SystemExit(main())
