import importlib.machinery
import importlib.util
from pathlib import Path
import rclpy

def test_monitor_can_initialize_without_system():
    script=Path(__file__).parents[1]/'scripts/readiness_monitor'
    loader=importlib.machinery.SourceFileLoader('readiness',str(script)); spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec); loader.exec_module(module)
    rclpy.init()
    node=module.Readiness()
    try:
        for _ in range(4): rclpy.spin_once(node,timeout_sec=.1)
        assert not node.amcl and not node.controllers
    finally: node.destroy_node(); rclpy.try_shutdown()
