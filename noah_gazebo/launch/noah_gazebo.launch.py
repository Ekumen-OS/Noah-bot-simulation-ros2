# Copyright 2022 Franco Cipollone

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Obtains share directory paths.
pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
pkg_noah_description = get_package_share_directory('noah_description')
pkg_noah_gazebo = get_package_share_directory('noah_gazebo')

def generate_launch_description():
    # Arguments
    rviz_argument = DeclareLaunchArgument('rviz', default_value='false',
                          description='Open RViz.')
    world_argument = DeclareLaunchArgument(
          'world',
          default_value='test.world',
          description='SDF world file name')
    verbose_argument = DeclareLaunchArgument('verbose', default_value='false',
                          description='Open Gazebo in verbose mode.')
    gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='true', description='Open Gazebo GUI.')

    # Includes gazebo_ros launch for gazebo
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
          launch_arguments = {
              'world': LaunchConfiguration('world'), # It is looking relative at GAZEBO_RESOURCE_PATH.
              'verbose': LaunchConfiguration('verbose'),
              'gui': LaunchConfiguration('gazebo_gui'),
          }.items()
    )

    # Include noah description launch file
    include_noah_description =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('noah_description'),
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
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_noah',
        arguments=['-topic', 'noah/robot_description', '-entity', 'noah', '-z', '0.1'],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_noah_gazebo, 'rviz', 'noah_gazebo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_argument,
        world_argument,
        verbose_argument,
        gazebo_gui,
        rviz,
        include_noah_description,
        include_gazebo,
        spawn_noah,
    ])
