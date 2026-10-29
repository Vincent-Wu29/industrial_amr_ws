#!/usr/bin/env python3
"""Copy completed Linux validation artifacts back beside the canonical source."""
import argparse
from collections import Counter
import json
import importlib.machinery
import importlib.util
from pathlib import Path
import shutil

def main():
    p=argparse.ArgumentParser(); p.add_argument('--source',required=True); p.add_argument('--name',required=True)
    args=p.parse_args(); root=Path(__file__).resolve().parents[1]
    if not args.name.replace('_','').replace('-','').isalnum(): p.error('name must be a simple identifier')
    source=Path(args.source).resolve(); destination=root/'artifacts/validation'/args.name
    if source==destination or source in destination.parents: p.error('source and destination must be separate')
    marker=destination/'evidence_source.json'
    if destination.exists() and any(destination.iterdir()):
        if not marker.exists() or json.loads(marker.read_text()).get('source')!=str(source):
            p.error('existing evidence belongs to another/unknown source; preserve it and choose a new --name')
    destination.mkdir(parents=True,exist_ok=True)
    marker.write_text(json.dumps({'source':str(source),'format_version':1},indent=2),encoding='utf-8')
    shutil.copytree(source,destination,dirs_exist_ok=True)
    for name in ('metrics.json','events.jsonl'):
        expected={str(f.relative_to(source)) for f in source.rglob(name)}
        copied={str(f.relative_to(destination)) for f in destination.rglob(name)}
        if copied!=expected: raise ValueError('stale measurement files in evidence; preserve it and choose a new --name')
    loader=importlib.machinery.SourceFileLoader('metrics',str(root/'src/amr_bringup/scripts/metrics_report'))
    spec=importlib.util.spec_from_loader(loader.name,loader); module=importlib.util.module_from_spec(spec); loader.exec_module(module)
    data=module.analyze(destination)
    resolution=Counter()
    for path in destination.rglob('metrics.json'):
        sample=json.loads(path.read_text())
        context='narrow navigation' if sample['scenario']=='narrow' else 'warehouse mission'
        resolution[(context,sample['profile'],sample.get('effective_profile',sample['profile']))]+=1
    data['configuration_resolution']=[{'context':c,'requested_profile':r,'effective_profile':e,'n':n}
                                     for (c,r,e),n in sorted(resolution.items())]
    note=''
    if any(r!=e for _,r,e in resolution):
        note='\n配置列profile为请求策略。仓库mission请求candidate时实际使用baseline，窄道navigation使用candidate数值；逐样本effective_profile和configuration_resolution记录实际配置。仓库回归不作为候选数值在仓库通用的证据。\n'
    report=destination.parent/(args.name+'_report.md'); report.write_text((note+'\n' if note else '')+module.report(data),encoding='utf-8')
    report.with_suffix('.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    for name in ['amr_final_build.log','amr_final_tests.log','amr_final_test_result.log']:
        path=Path('/tmp')/name
        if path.exists(): shutil.copy2(path,destination.parent/name)
    for parent in source.parents:
        if (parent/'.industrial-amr-owned').exists():
            for name in ['latest_unit.log','mapping_fix_build.log','final_test_result.log','final_unit.log','mission_retest.log','clearance_build.log','profile_final_retest.log','bringup_final_retest.log']:
                path=parent/'artifacts'/name
                if path.exists(): shutil.copy2(path,destination.parent/name)
            break
    # A formal run's own test report supersedes older cached audit logs.
    for name,dest in [('test_result.log','final_test_result.log'),('unit.log','final_unit.log')]:
        path=source/name
        if path.exists(): shutil.copy2(path,destination.parent/dest)
    print(report); print(report.read_text(encoding='utf-8'))
if __name__=='__main__': main()
