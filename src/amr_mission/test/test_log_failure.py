import os
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory as share

def test_unwritable_artifact_directory_fails_before_acceptance(tmp_path):
    blocked=tmp_path/'blocked'; blocked.mkdir(mode=0o500)
    env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(210+os.getpid()%10)
    command=['ros2','run','amr_mission','mission_manager','--ros-args',
        '-p',f"stations_file:={share('amr_mission')}/config/stations.yaml",
        '-p',f"map_file:={share('amr_navigation')}/maps/warehouse_v1.yaml",'-p',f'artifacts_dir:={blocked}']
    try:
        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=10)
        assert result.returncode==1
        assert 'Permission denied' in result.stderr or 'Permission denied' in result.stdout
        assert not list(blocked.iterdir())
    finally: blocked.chmod(0o700)
