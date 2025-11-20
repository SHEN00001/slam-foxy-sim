import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 获取包路径
    pkg_share = get_package_share_directory('robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. 获取URDF(Xacro)文件路径
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'delivery_robot.urdf')

    # 3. Robot State Publisher
    #    (加载xacro文件并发布 /robot_description 话题)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path).read()}]
    )

    # 4. Gazebo (启动Gazebo服务器和客户端)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # 5. Spawn Entity (在Gazebo中生成机器人)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'delivery_robot',
                   '-x', '0',
                   '-y', '0',
                   '-z', '0.1'],
        output='screen'
    )

    # 6. RViz2 (可选，用于可视化)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot.rviz') # 您需要创建这个文件
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    # (注意：您需要先运行一次RViz2，添加RobotModel、TF、LaserScan、PointCloud2等，然后保存配置到 delivery_robot_description/config/robot.rviz)

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        rviz_node
    ])