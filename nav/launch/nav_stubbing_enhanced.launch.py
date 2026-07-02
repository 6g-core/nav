import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    go2_base_node = Node(
        package='go2_base',
        executable='go2_base',
        name='go2_base',
        output='screen'
    )
    go2_state_node = Node(
        package='simple_controls',
        executable='log_state',
        name='log_state',
        output='screen'
    )

    # go2_core_dir = get_package_share_directory('go2_base')
    go2_slam_dir = get_package_share_directory('go2_slam')
    go2_perception_dir = get_package_share_directory('go2_perception')
    go2_slam_dir = get_package_share_directory('go2_slam')
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_slam_dir, 'launch', 'go2_slamtoolbox.launch.py')
        ])
    )

    pointcloud_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(go2_perception_dir, 'launch', 'go2_pointcloud_process.launch.py')
        ])
    )
    nav_stubbing_enhanced_node = Node(
        package='nav',
        executable='nav_stubbing_enhanced',
        name='nav_stubbing_enhanced',
        output='screen',
    )

    ld.add_action(go2_base_node)
    ld.add_action(go2_state_node)
    # ld.add_action(pointcloud_process_launch)
    # ld.add_action(slam_toolbox_launch)
    ld.add_action(nav_stubbing_enhanced_node)
    ld.add_action

    return ld