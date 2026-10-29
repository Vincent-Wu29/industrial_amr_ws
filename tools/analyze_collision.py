#!/usr/bin/env python3
"""Attribute geometric collision entries using recorded truth and fixture poses."""
import argparse
import ast
import json
import math
from pathlib import Path
from generate_assets import BOXES

def main():
    p=argparse.ArgumentParser(); p.add_argument('run_dir'); args=p.parse_args(); folder=Path(args.run_dir)
    root=Path(__file__).resolve().parents[1]
    source=ast.parse((root/'src/amr_simulation/scripts/truth_monitor').read_text(encoding='utf-8'))
    function=next(node for node in source.body if isinstance(node,ast.FunctionDef) and node.name=='intersects')
    namespace={'math':math}; exec(compile(ast.Module(body=[function],type_ignores=[]),'collision_geometry','exec'),namespace)
    intersects=namespace['intersects']
    truth=[json.loads(line) for line in (folder/'truth.jsonl').read_text().splitlines()]
    obstacle_path=folder/'obstacle_truth.jsonl'
    if not obstacle_path.exists(): obstacle_path=folder/'obstacles.jsonl'
    obstacles=[json.loads(line) for line in obstacle_path.read_text().splitlines()] if obstacle_path.exists() else []
    contact=False; results=[]
    for index,sample in enumerate(truth):
        if sample['collision'] and not contact:
            boxes={f'static_{i}':box[:4] for i,box in enumerate(BOXES)}
            for event in obstacles:
                if event['sim_s']>sample['sim_s']+.05: break
                name=event['fixture']; sx,sy=(.2,2.4) if name in ('enclosure_west','enclosure_east') else (2.4,.2) if name in ('enclosure_north','enclosure_south') else (.6,.6)
                boxes[name]=(event['x'],event['y'],sx,sy)
            previous=truth[max(0,index-1)]; delta=sample['sim_s']-previous['sim_s']
            offenders=[name for name,box in boxes.items() if intersects(sample['x'],sample['y'],sample['yaw'],box)]
            results.append({'sample':sample,'offenders':offenders,'fixture':boxes.get('scenario_box'),
                'estimated_truth_speed':math.hypot(sample['x']-previous['x'],sample['y']-previous['y'])/delta if delta>0 else None})
        contact=sample['collision']
    print(json.dumps(results,indent=2))
if __name__=='__main__': main()
