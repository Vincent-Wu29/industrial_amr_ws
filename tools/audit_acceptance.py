#!/usr/bin/env python3
"""Check final sample counts, provenance, physical assertions and paired results."""
import argparse
from datetime import datetime
import hashlib
from collections import Counter
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import re
import yaml
from fill_normal_samples import inspect as inspect_normal


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--input',required=True)
    parser.add_argument('--require-performance',action='store_true',help='Fail unless the registered P95 target also passes')
    args=parser.parse_args(); folder=Path(args.input).resolve(); root=Path(__file__).resolve().parents[1]
    issues=[]; summaries={}; fingerprints=set(); common_fingerprints=set(); middlewares=set(); params={}
    registration_path=folder/'m4_registration.json'
    registration=json.loads(registration_path.read_text()) if registration_path.exists() else {}
    if not registration: issues.append('missing frozen M4 registration')
    allowed='amr_navigation/config/candidate.yaml'
    harness='amr_bringup/scripts/scenario_runner'
    revisions=registration.get('harness_revision',{})
    resource='amr_navigation/test/test_resources.py'
    resource_revision=registration.get('resource_test_revision',{})
    exempt={allowed,harness,resource}
    if registration.get('candidate_scope')!={'scenario':'narrow','world':'narrow_aisle','mode':'navigation','other_scenarios':'baseline'}:
        issues.append('registration must bind the candidate to the fixed narrow navigation experiment')
    cases=yaml.safe_load((root/'src/amr_bringup/config/suites.yaml').read_text())['m3_regression']
    expected={'m3_happy':{('happy','baseline'):30},
              'm3_regression':{(case,'baseline'):5 for case in cases},
              'm4_preflight':{('narrow',profile):1 for profile in ('baseline','candidate')},
              'm4':{('narrow',profile):30 for profile in ('baseline','candidate')},
              'm4_regression':{(case,'candidate'):1 for case in cases}}
    if (folder/'obstacle_probe').exists():
        expected['obstacle_probe']={(case,'baseline'):5 for case in ('crossing','temporary')}
    for stage,wanted in expected.items():
        paths=sorted((folder/stage).rglob('metrics.json')); records=[]; seeds={}
        for path in paths:
            sample=json.loads(path.read_text()); records.append(sample)
            if sample.get('scenario')=='narrow':
                smoke=path.with_name('navigation_smoke.json')
                actual=json.loads(smoke.read_text()) if smoke.exists() else []
                wanted_goals=registration.get('staging',[])+[registration.get('timed_goal')]
                if [[goal.get('x'),goal.get('y')] for goal in actual]!=wanted_goals:
                    issues.append(str(path.relative_to(folder))+': observed goals differ from registered staging/timed goal')
            key=(sample['scenario'],sample['profile']); seeds.setdefault(key,[]).append(sample['seed'])
            if not sample['passed'] and stage!='m3_happy': issues.append(str(path.relative_to(folder))+': assertion failed')
            if sample['collision_count']!=0: issues.append(str(path.relative_to(folder))+': collision count is not verified zero')
            if sample['scenario'] not in ('guard_exit','nav_exit','clock_reset') and (stage!='m3_happy' or sample['passed']):
                if not sample.get('truth_valid') or not sample.get('physical_stop_verified'):
                    issues.append(str(path.relative_to(folder))+': missing truth or stationary proof')
            manifest=path.with_name('manifest.yaml')
            if not manifest.exists(): issues.append(str(path.relative_to(folder))+': missing manifest'); continue
            provenance=yaml.safe_load(manifest.read_text())
            command=provenance.get('launch_command',[])
            navigation=sample.get('scenario')=='narrow'
            if (f'mode:={"navigation" if navigation else "mission"}' not in command or
                    f'world:={"narrow_aisle" if navigation else "warehouse"}' not in command):
                issues.append(str(manifest.relative_to(folder))+': actual launch world/mode differs from registration')
            factor=3.0 if stage in ('m4_preflight','m4') else 1.0
            if not provenance.get('headless') or provenance.get('target_real_time_factor')!=factor:
                issues.append(str(manifest.relative_to(folder))+': headless or target RTF differs from registration')
            began=datetime.fromisoformat(provenance['utc'].replace('Z','+00:00')).timestamp()
            if path.stat().st_mtime-began>1500:
                issues.append(str(path.relative_to(folder))+': real UTC span exceeds worker ceiling; possible host suspension')
            middlewares.add(provenance.get('rmw'))
            hashes=provenance.get('source_hashes',{})
            if not hashes: issues.append(str(manifest.relative_to(folder))+': missing source hashes')
            else:
                fingerprints.add(tuple(sorted(hashes.items())))
                common_fingerprints.add(tuple(sorted((k,v) for k,v in hashes.items() if k not in exempt)))
                if hashes.get(harness) not in (revisions.get('old_sha256'),revisions.get('new_sha256')):
                    issues.append(str(manifest.relative_to(folder))+': unregistered harness version')
                if stage.startswith('m4') and hashes.get(harness)!=revisions.get('new_sha256'):
                    issues.append(str(manifest.relative_to(folder))+': M4 did not use the corrected registered staging')
                if hashes.get(resource) not in (resource_revision.get('old_sha256'),resource_revision.get('new_sha256')):
                    issues.append(str(manifest.relative_to(folder))+': unregistered resource test version')
                if stage.startswith('m4') and hashes.get(resource)!=resource_revision.get('new_sha256'):
                    issues.append(str(manifest.relative_to(folder))+': M4 resource constraints differ from delivered profile')
            effective=path.with_name('nav2.yaml').read_text()
            resolved=provenance.get('effective_profile',sample.get('effective_profile',sample['profile']))
            expected_profile=sample['profile'] if sample['scenario']=='narrow' else 'baseline'
            if resolved!=expected_profile or sample.get('effective_profile',resolved)!=resolved:
                issues.append(str(path.relative_to(folder))+': candidate policy escaped its registered map/mode scope')
            if stage.startswith('m4') and ('effective_profile' not in provenance or 'effective_profile' not in sample):
                issues.append(str(path.relative_to(folder))+': new cohort must record requested and effective profiles')
            params.setdefault(resolved,set()).add(effective)
        counts=Counter((s['scenario'],s['profile']) for s in records)
        if dict(counts)!=wanted: issues.append(f'{stage}: expected {wanted}, got {dict(counts)}')
        for key,values in seeds.items():
            start=46 if stage=='obstacle_probe' else registration.get('preflight_seed',62) if stage=='m4_preflight' else 42
            if sorted(values)!=list(range(start,start+wanted.get(key,0))):
                issues.append(f'{stage}/{key}: incomplete or duplicate registered seeds')
        summaries[stage]={'n':len(records),'passed':sum(bool(s['passed']) for s in records),
                          'counts':{str(k):v for k,v in counts.items()},
                          'effective_profiles':dict(Counter(s.get('effective_profile',s['profile']) for s in records))}
    normal_records,normal=inspect_normal(folder/'m3_happy')
    if not normal['meets_target']: issues.append('normal cohort does not meet design threshold >=95% / zero observed contacts / bounded HOLD')
    m2_passed=all(seed in normal_records and normal_records[seed][0]['passed'] for seed in range(42,52))
    if not m2_passed:
        issues.append('M2 first ten registered transports are not all successful')
    summaries['m3_happy']['normal_sampling_audit']=normal
    if len(common_fingerprints)!=1: issues.append(f'common source changed beyond registered candidate and narrow harness coordinates: {len(common_fingerprints)} versions')
    if len(fingerprints)>2: issues.append(f'expected at most original and registered candidate revision, found {len(fingerprints)}')
    if registration.get('allowed_source_revision')!=allowed: issues.append('unexpected source revision exception')
    current={}
    for file in (root/'src').rglob('*'):
        if file.is_file() and '__pycache__' not in file.parts and file.suffix!='.pyc':
            current[str(file.relative_to(root/'src'))]=hashlib.sha256(file.read_bytes()).hexdigest()
    current_common=tuple(sorted((k,v) for k,v in current.items() if k not in exempt))
    if common_fingerprints!={current_common}: issues.append('delivered common source differs from measured source')
    if tuple(sorted(current.items())) not in fingerprints: issues.append('delivered candidate source has no measured provenance')
    old_harness=folder/'harness_v6.py'
    if not old_harness.exists():
        if len(fingerprints)>1: issues.append('missing old harness snapshot for exact revision check')
    else:
        old_text=old_harness.read_text()
        expected_text=old_text.replace('navigate(7.,12.4); navigate(7.5,12.4)',
                                      'navigate(6.5,12.4); navigate(7.,12.4)').replace('navigate(12.5,12.4)','navigate(13.,12.4)')
        expected_text=expected_text.replace('def stop(process):',
            "def effective_profile(profile,scenario):\n    return profile if scenario=='narrow' else 'baseline'\n\n\ndef stop(process):",1).replace(
            "    scenario=args.case; navigation=scenario=='narrow'\n",
            "    scenario=args.case; navigation=scenario=='narrow'\n    resolved_profile=effective_profile(args.profile,scenario)\n").replace(
            "if args.profile=='candidate':","if resolved_profile=='candidate':").replace(
            "'profile':args.profile,'seed':args.seed","'profile':args.profile,'effective_profile':resolved_profile,'seed':args.seed")
        if (old_text==expected_text or expected_text!=(root/'src'/harness).read_text() or
                hashlib.sha256(old_harness.read_bytes()).hexdigest()!=revisions.get('old_sha256') or
                current.get(harness)!=revisions.get('new_sha256') or revisions.get('path')!=harness):
            issues.append('harness changed beyond registered coordinates, map policy resolution and its provenance fields')
    old_resource=folder/'resources_v8.py'
    if not old_resource.exists():
        if len(fingerprints)>1: issues.append('missing original resource test for exact revision check')
    else:
        old_text=old_resource.read_text()
        expected_text=old_text.replace("assert set(params)=={'inflation_layer'}", "assert set(params)=={'inflation_layer','obstacle_layer'}").replace(
            "assert params['inflation_layer']['cost_scaling_factor']>0",
            "assert params['inflation_layer']['cost_scaling_factor']>0\n    assert params['obstacle_layer']=={'scan':{'obstacle_max_range':1.0}}")
        expected_text+='\n\ndef test_candidate_policy_is_scoped_to_narrow_navigation():\n    import importlib.machinery\n    import importlib.util\n    path=ROOT.parent/\'amr_bringup/scripts/scenario_runner\'\n    loader=importlib.machinery.SourceFileLoader(\'profile_scope\',str(path))\n    spec=importlib.util.spec_from_loader(loader.name,loader)\n    module=importlib.util.module_from_spec(spec); loader.exec_module(module)\n    assert module.effective_profile(\'candidate\',\'narrow\')==\'candidate\'\n    assert module.effective_profile(\'baseline\',\'narrow\')==\'baseline\'\n    cases=yaml.safe_load((ROOT.parent/\'amr_bringup/config/suites.yaml\').read_text())[\'m3_regression\']\n    for scenario in cases:\n        assert module.effective_profile(\'candidate\',scenario)==\'baseline\'\n        assert module.effective_profile(\'baseline\',scenario)==\'baseline\'\n'
        if (old_text==expected_text or expected_text!=(root/'src'/resource).read_text() or
                hashlib.sha256(old_resource.read_bytes()).hexdigest()!=resource_revision.get('old_sha256') or
                current.get(resource)!=resource_revision.get('new_sha256') or resource_revision.get('path')!=resource):
            issues.append('resource tests differ beyond registered global-range and policy-scope assertions')
    if None in middlewares or len(middlewares)!=1: issues.append('missing or mixed RMW implementations across stages')
    for profile,versions in params.items():
        if len(versions)!=1: issues.append(f'{profile}: multiple effective navigation configurations')
    if all(len(params.get(p,set()))==1 for p in ('baseline','candidate')):
        baseline=yaml.safe_load(next(iter(params['baseline']))); candidate=yaml.safe_load(next(iter(params['candidate'])))
        cost=lambda config:config['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
        if cost(baseline)['cost_scaling_factor']!=registration.get('baseline_global_cost_scaling') or cost(candidate)['cost_scaling_factor']!=registration.get('candidate_global_cost_scaling'):
            issues.append('global cost scaling differs from frozen registration')
        cost(candidate)['cost_scaling_factor']=3.0
        scan=lambda config:config['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']
        if scan(baseline)['obstacle_max_range']!=registration.get('baseline_global_obstacle_range') or scan(candidate)['obstacle_max_range']!=registration.get('candidate_global_obstacle_range'):
            issues.append('global marking range differs from frozen registration')
        scan(candidate)['obstacle_max_range']=scan(baseline)['obstacle_max_range']
        if baseline!=candidate: issues.append('candidate differs beyond registered global scaling and marking range')
    unit=folder/'test_result.log'
    match=re.search(r'Summary: (\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped',unit.read_text()) if unit.exists() else None
    if not match or int(match[1])<43 or any(int(match[i]) for i in (2,3,4)): issues.append('unit summary missing or not all 43 checks passing')
    m1=folder/'m1/summary.json'
    navigation=json.loads(m1.read_text()) if m1.exists() else {}
    if not navigation.get('passed'): issues.append('M1 final navigation/mapping validation missing or failed')
    for name,count in [('fixed_map',15),('saved_map',3)]:
        group=navigation.get(name,{})
        if len(group.get('goals',[]))!=count or not all(g['success'] for g in group.get('goals',[])) or group.get('collision_count')!=0:
            issues.append(f'M1/{name}: expected {count} successful goals and zero collisions')
    if navigation.get('mapping',{}).get('collision_count')!=0: issues.append('M1/mapping: no verified zero-collision trace')
    loader=importlib.machinery.SourceFileLoader('acceptance_metrics',str(root/'src/amr_bringup/scripts/metrics_report'))
    spec=importlib.util.spec_from_loader(loader.name,loader); metrics=importlib.util.module_from_spec(spec); loader.exec_module(metrics)
    totals=metrics.analyze(folder); issues.extend(totals['integrity_problems'])
    paired=metrics.analyze(folder/'m4')['scenario_groups']; comparison=None
    if all('narrow/'+p in paired for p in ('baseline','candidate')):
        comparison=metrics.comparison(paired['narrow/baseline'],paired['narrow/candidate'],'sim')
    result={'complete_and_consistent':not issues,'issues':issues,'stages':summaries,
            'source_versions':len(fingerprints),'common_source_versions':len(common_fingerprints),
            'allowed_source_revision':allowed,'exact_harness_revision':revisions,'exact_resource_test_revision':resource_revision,
            'registration':registration,'middleware':sorted(m for m in middlewares if m),
            'unit_summary':match.group(0) if match else None,'m2_first_ten_passed':m2_passed,'m4_comparison':comparison}
    result['overall_acceptance_passed']=not issues and bool(comparison and comparison['meets_target'])
    destination=folder/'acceptance_audit.json'; destination.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    passed=result['overall_acceptance_passed'] if args.require_performance else not issues
    return 0 if passed else 1


if __name__=='__main__': raise SystemExit(main())
