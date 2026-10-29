import importlib.machinery
import importlib.util
import json
from pathlib import Path

def load():
    path=Path(__file__).parents[1]/'scripts/metrics_report'
    loader=importlib.machinery.SourceFileLoader('metrics',str(path)); spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec); loader.exec_module(module); return module

def test_conservative_denominators(tmp_path):
    m=load(); events=[]
    for i,(task,kind,state) in enumerate([('a','admission_accepted','IDLE'),('a','terminal','COMPLETED'),
            ('b','admission_accepted','IDLE'),('b','terminal','CANCELED'),('c','admission_accepted','IDLE'),
            ('d','admission_rejected','IDLE')]):
        events.append({'run_id':'r','task_id':task,'event_seq':i+1,'event_type':kind,'state':state,'error_code':0,
            'elapsed_sim_s':12,'elapsed_wall_s':24})
    (tmp_path/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
    data=m.analyze(tmp_path)
    assert data['accepted_tasks']==3 and data['task_success_rate']==1/3
    assert data['task_outcomes']=={'COMPLETED':1,'CANCELED':1,'INTERRUPTED':1}
    assert data['successful_task_sim_s']=={'n':1,'mean':12,'median':12,'p95':12}

def test_p95_and_empty():
    m=load(); assert m.stats([])['p95'] is None
    assert m.stats(list(range(1,21)))['p95']==19

def test_invalid_and_duplicate_logs_are_exposed(tmp_path):
    m=load(); e={'run_id':'r','task_id':'a','event_seq':1,'event_type':'admission_accepted','state':'IDLE','error_code':0}
    (tmp_path/'events.jsonl').write_text(json.dumps(e)+'\n'+json.dumps(e)+'\n'+'invalid json')
    assert len(m.analyze(tmp_path)['integrity_problems'])==3

def test_smoke_and_unknown_navigation_attempts_are_counted(tmp_path):
    m=load()
    event={'run_id':'r','task_id':'a','event_seq':1,'event_type':'nav_attempt','nav_sequence':1,'ros_stamp':1,'wall_offset_s':1}
    (tmp_path/'events.jsonl').write_text(json.dumps(event))
    (tmp_path/'navigation_smoke.json').write_text(json.dumps([
        {'success':True,'outcome':'4','elapsed_sim_s':5,'elapsed_wall_s':6},
        {'success':False,'outcome':'rejected'}]))
    data=m.analyze(tmp_path)
    assert data['navigation_attempts']==3 and data['navigation_success_rate']==1/3
    assert data['navigation_outcomes']=={'UNKNOWN':1,'SUCCEEDED':1,'FAILED':1}
    assert data['successful_navigation_sim_s']['p95']==5

def test_speedup_cannot_override_failed_or_unpaired_samples():
    m=load()
    a={'n':30,'passed':30,'collisions':0,'unverified_truth':0,'seeds':list(range(42,72)),
       'succeeded':30,'success_wall_s':{'p95':10},'success_sim_s':{'p95':10}}
    b={**a,'success_wall_s':{'p95':8}}
    assert m.comparison(a,b,'wall')['meets_target']
    assert not m.comparison(a,{**b,'collisions':1},'wall')['meets_target']
    assert not m.comparison(a,{**b,'passed':29},'wall')['meets_target']
    assert not m.comparison(a,{**b,'seeds':[42]*30},'wall')['meets_target']
    assert not m.comparison(a,{**b,'success_wall_s':{'p95':9}},'wall')['meets_target']
    assert not m.comparison(a,{**b,'success_sim_s':{'p95':11}},'wall')['meets_target']
