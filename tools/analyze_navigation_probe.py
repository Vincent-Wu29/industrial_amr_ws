#!/usr/bin/env python3
"""Compare native global plans/costmap snapshots against independent Gazebo truth."""
import argparse
import bisect
import json
import math
from pathlib import Path


def load(folder,name):
    return [json.loads(line) for line in (folder/(name+'.jsonl')).read_text().splitlines()]


def corridor_costs(snapshot):
    rows=snapshot['rows']; resolution=snapshot['resolution']
    columns=[i for i in range(len(rows[0])) if 8.5<=snapshot['x0']+(i+.5)*resolution<=11.5]
    band=[j for j in range(len(rows)) if 12<=snapshot['y0']+(j+.5)*resolution<=12.8]
    interior=[j for j in band if 12.15<=snapshot['y0']+(j+.5)*resolution<=12.65]
    minima=[min(rows[j][i] for j in band) for i in columns]
    return {'lethal_columns':sum(v>=253 for v in minima),'max_min_column_cost':max(minima),
            'occupied_cells_in_empty_interior':sum(rows[j][i]==254 for j in interior for i in columns)}


def analyze(folder):
    plans=load(folder,'plans'); costs=load(folder,'global_costmaps'); amcl=load(folder,'amcl'); truth=load(folder,'truth')
    if not costs or not truth: raise ValueError('native costmap and Gazebo truth required')
    timed=[p for p in plans if p['points'] and abs(p['points'][-1][0]-13)<.2 and abs(p['points'][-1][1]-12.4)<.2]
    times=[c['sim_s'] for c in costs]; records=[]
    for plan in timed:
        index=max(0,bisect.bisect_right(times,plan['sim_s'])-1)
        nearby=[p for p in plan['points'] if 8.5<=p[0]<=11.5]
        route='outside_roi' if not nearby else 'corridor' if all(12<=p[1]<=12.8 for p in nearby) else 'bypass'
        records.append({'sim_s':plan['sim_s'],'costmap_index':index,'costmap_sim_s':costs[index]['sim_s'],
                        'route':route,**corridor_costs(costs[index])})
    truth_times=[t['source_sim_s'] for t in truth]; errors=[]
    for pose in amcl:
        index=min(len(truth)-1,bisect.bisect_left(truth_times,pose['sim_s'])); actual=truth[index]
        errors.append({'sim_s':pose['sim_s'],'dx':pose['x']-actual['x'],'dy':pose['y']-actual['y'],
                       'yaw_delta':math.remainder(pose['yaw']-actual['yaw'],2*math.pi)})
    during=[e for e in errors if timed and e['sim_s']>=timed[0]['sim_s']]
    return {'folder':str(folder),'metrics':json.loads((folder/'metrics.json').read_text()),
            'plan_count':len(plans),'costmap_count':len(costs),'timed_plan_costs':records,
            'max_lethal_columns':max((r['lethal_columns'] for r in records),default=None),
            'max_interior_occupied_cells':max((r['occupied_cells_in_empty_interior'] for r in records),default=None),
            'max_timed_amcl_xy_error_m':max((math.hypot(e['dx'],e['dy']) for e in during),default=None),
            'max_pre_goal_amcl_xy_error_m':max((math.hypot(e['dx'],e['dy']) for e in errors if e not in during),default=None)}


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--input',required=True)
    args=parser.parse_args(); root=Path(args.input).resolve()
    results=[analyze(p) for p in sorted(root.iterdir()) if p.is_dir() and (p/'metrics.json').exists()]
    (root/'probe_analysis.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps([{k:r[k] for k in ('folder','max_lethal_columns','max_interior_occupied_cells',
        'max_timed_amcl_xy_error_m','max_pre_goal_amcl_xy_error_m')} for r in results],indent=2))


if __name__=='__main__': main()
