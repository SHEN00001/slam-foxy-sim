""" 启动Gazebo加载指定world文件 """

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    local_model_path = '/home/shen/workspace/ROS2/robot_ws/src/robot_description/world/car_junction'

    world_file_path = os.path.join(local_model_path, 'world', 'city.world')

    # 1. 获取当前系统中已有的 GAZEBO_MODEL_PATH
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    
    # 2. 如果已有路径，则用冒号(:)连接；如果没有，直接设置为新路径
    if existing_model_path:
        new_model_path = existing_model_path + ':' + local_model_path
    else:
        new_model_path = local_model_path

    # 3. 使用 SetEnvironmentVariable 设置新的完整路径
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_file_path 
        }.items()
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo_launch
    ])
