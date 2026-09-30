from launch import LaunchDescription
from launch_ros.actions import Node

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

    http_control_server_node = Node(
        package='go2_http_control',
        executable='http_control_server',
        name='http_control_server',
        output='screen',
    )

    # ========== 新增：手柄监听节点 ==========
    wireless_controller_node = Node(
        package='go2_wireless_controller',
        executable='wireless_controller_node',
        name='wireless_controller',
        output='screen',
    )
    # ========================================

    ld.add_action(go2_base_node)
    ld.add_action(go2_state_node)
    ld.add_action(http_control_server_node)
    ld.add_action(wireless_controller_node)

    return ld
