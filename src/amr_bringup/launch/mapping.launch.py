from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    args={'world':'mini_warehouse','gui':'false','rviz':'false','use_sim_time':'true','artifacts_dir':'artifacts','seed':'42','real_time_factor':'1.0'}
    path=Path(get_package_share_directory('amr_bringup'))/'launch/system.launch.py'
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments={**{k:LaunchConfiguration(k) for k in args},'mode':'mapping'}.items())])
