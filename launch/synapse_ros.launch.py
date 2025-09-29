import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('host', default_value='192.0.2.1',
                          description='port for cerebri'),
    DeclareLaunchArgument('port', default_value='4242',
                          description='tcp port for cerebri'),
    DeclareLaunchArgument('port_srv', default_value='4242',
                          description='tcp port for server'),
    DeclareLaunchArgument('rpmsg_dev', default_value='',
                          description=('RPMsg character device. When '
                                       'configured this will be used instead '
                                       'of sockets')),
    DeclareLaunchArgument('mode', default_value='real',
                          choices=['real', 'hil', 'sil'],
                          description='communication mode, (real-life [rea]/ hardware-in-the-loop [hil]/ software-in-the-loop [sil])'),

    DeclareLaunchArgument('namespace', default_value='cerebri',
                          description='node namespace'),

    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():

    # Launch configurations
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    port_srv = LaunchConfiguration('port_srv')
    mode = LaunchConfiguration('mode')
    rpmsg_dev = LaunchConfiguration('rpmsg_dev')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    synapse_ros = Node(
        #prefix='xterm -e gdb --args',
        namespace=namespace,
        package='synapse_ros',
        executable='synapse_ros',
        parameters=[{
            'host': host,
            'port': port,
            'port_srv': port_srv,
            'mode': mode,
            'rpmsg_dev': rpmsg_dev,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        remappings=[
            ('in/cmd_vel', '/cmd_vel')
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        on_exit=Shutdown(),
        #prefix=['xterm -e gdb -ex=r --args'],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(synapse_ros)
    return ld

