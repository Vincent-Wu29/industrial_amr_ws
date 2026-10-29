import importlib.util
import importlib.machinery
from pathlib import Path
import math
import xml.etree.ElementTree as ET

def test_oriented_collision_monitor():
    script=Path(__file__).parents[1]/'scripts/truth_monitor'
    loader=importlib.machinery.SourceFileLoader('truth',str(script)); spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec); loader.exec_module(module)
    assert module.intersects(0,0,0,(.3,0,.3,.3))
    assert not module.intersects(0,0,0,(1,0,.3,.3))
    assert module.intersects(0,0,math.pi/4,(.18,.18,.1,.1))
    assert not module.intersects(0,0,math.pi/4,(.2,.2,.1,.1))

def test_world_has_physical_obstacles_and_sensors_system():
    folder=Path(__file__).parents[1]/'worlds'
    for path in folder.glob('*.sdf'):
        root=ET.parse(path).getroot(); assert root.find('.//model[@name="scenario_box"]/link/collision') is not None
        assert root.find('.//plugin[@name="gz::sim::systems::Sensors"]') is not None
