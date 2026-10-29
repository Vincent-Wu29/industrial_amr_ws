from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).parents[1]

def test_stamped_pipeline_and_footprint():
    config=yaml.safe_load((ROOT/'config/nav2.yaml').read_text())
    for name in ('controller_server','behavior_server'): assert config[name]['ros__parameters']['enable_stamped_cmd_vel']
    for name in ('local_costmap','global_costmap'):
        c=config[name][name]['ros__parameters']; assert c['robot_base_frame']=='base_footprint'
        assert yaml.safe_load(c['footprint'])==[[.2,.15],[.2,-.15],[-.2,-.15],[-.2,.15]]

def test_world_map_and_bt_resources():
    map_info=yaml.safe_load((ROOT/'maps/warehouse_v1.yaml').read_text())
    assert map_info['map_id']=='warehouse_v1' and map_info['resolution']==.05
    data=(ROOT/'maps'/map_info['image']).read_bytes(); assert data.startswith(b'P5\n400 300\n255\n')
    root=ET.parse(ROOT/'behavior_trees/navigate.xml').getroot()
    assert root.find('.//RecoveryNode').attrib['number_of_retries']=='2'
    assert root.find('.//BackUp') is None

def test_experimental_profile_cannot_override_local_safety():
    candidate=yaml.safe_load((ROOT/'config/candidate.yaml').read_text())
    assert set(candidate)=={'global_costmap'}
    params=candidate['global_costmap']['global_costmap']['ros__parameters']
    assert set(params)=={'inflation_layer','obstacle_layer'}
    assert set(params['inflation_layer'])=={'cost_scaling_factor'}
    assert params['inflation_layer']['cost_scaling_factor']>0
    assert params['obstacle_layer']=={'scan':{'obstacle_max_range':1.0}}


def test_candidate_policy_is_scoped_to_narrow_navigation():
    import importlib.machinery
    import importlib.util
    path=ROOT.parent/'amr_bringup/scripts/scenario_runner'
    loader=importlib.machinery.SourceFileLoader('profile_scope',str(path))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec); loader.exec_module(module)
    assert module.effective_profile('candidate','narrow')=='candidate'
    assert module.effective_profile('baseline','narrow')=='baseline'
    cases=yaml.safe_load((ROOT.parent/'amr_bringup/config/suites.yaml').read_text())['m3_regression']
    for scenario in cases:
        assert module.effective_profile('candidate',scenario)=='baseline'
        assert module.effective_profile('baseline',scenario)=='baseline'
