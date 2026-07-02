import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
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

    go2_slam_dir = get_package_share_directory('go2_slam')
    go2_perception_dir = get_package_share_directory('go2_perception')

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

    nav_stubbing_enhanced_planB_node = Node(
        package='nav',
        executable='nav_stubbing_enhanced_planB',
        name='nav_stubbing_enhanced_planB',
        output='screen',
    )

    # ========== 新增：手柄监听节点 ==========
    start_button_sender_node = Node(
        package='go2_start_button_sender',
        executable='start_button_node',
        name='start_button_sender',
        output='screen',
    )
    # ========================================

    ld.add_action(go2_base_node)
    ld.add_action(go2_state_node)
    # ld.add_action(pointcloud_process_launch)
    # ld.add_action(slam_toolbox_launch)
    ld.add_action(nav_stubbing_enhanced_planB_node)
    ld.add_action(start_button_sender_node)   # ← 注册新手柄节点

    return ld
