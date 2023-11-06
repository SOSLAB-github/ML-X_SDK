import os
import launch

from launch_ros.actions import Node

def generate_launch_description():

    ml_node = Node(
        package="ml",
        executable="ml_node",
        name="ml_node",
        output="screen",
        emulate_tty=True,
        parameters = [
            {'ip_address_device': '192.168.1.10'},
            {'ip_port_device': 2000},
            {'ip_address_pc': '192.168.1.15'},
            {'ip_port_pc': 2000},
        ],
    )

    rviz_config_file = os.path.join('../rviz', 'config.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments = ['-d', rviz_config_file],
    )

    ld = launch.LaunchDescription()
    ld.add_action( ml_node )
    ld.add_action( rviz_node )
    
    return ld
