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

    ld = launch.LaunchDescription()
    ld.add_action( ml_node )
    
    return ld
