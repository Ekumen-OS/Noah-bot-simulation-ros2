# Copyright 2022 Franco Cipollone


"""Launch Ign Gazebo with a world that has the Noah."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_noah_description = get_package_share_directory('noah_description')
    pkg_noah_ignition = get_package_share_directory('noah_ignition')

    # Arguments
    rviz_argument = DeclareLaunchArgument('rviz', default_value='false',
                          description='Open RViz.')
    world_argument = DeclareLaunchArgument(
          'world',
          default_value='test.world',
          description='SDF world file name')
    verbose_argument = DeclareLaunchArgument('verbose', default_value='false',
                          description='Open Gazebo in verbose mode.')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),
        ),
          launch_arguments = {
              'ign_args': LaunchConfiguration('world'),
          }.items()
    )

    # Include noah description launch file
    include_noah_description =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_noah_description,
                'launch',
                'noah_description.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'False',
            'rsp': 'True',
            'jsp': 'False', # It could be enabled though
        }.items()
    )

    # Spawn noah
    spawn_noah = Node(
        package='ros_ign_gazebo',
        executable='create',
        name='spawn_noah',
                 arguments=[
                    '-name', 'noah',
                    '-x', '0.0',
                    '-z', '0.1',
                    '-Y', '0.0',
                    '-topic', "/noah/robot_description"],
        output='screen')

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/noah/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/noah/laser_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/noah/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_noah_ignition, 'rviz', 'noah_ignition.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_argument,
        world_argument,
        verbose_argument,
        include_noah_description,
        gazebo,
        spawn_noah,
        bridge,
        rviz
    ])
