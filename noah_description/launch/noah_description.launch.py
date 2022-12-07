# Copyright 2022 Franco Cipollone

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Arguments
    rviz_argument = DeclareLaunchArgument('rviz', default_value='true',
                          description='Open RViz.')
    rsp_argument = DeclareLaunchArgument('rsp', default_value='true',
                          description='Run robot state publisher node.')
    jsp_argument = DeclareLaunchArgument('jsp', default_value='true',
                          description='Run joint state publisher node.')

    # Obtains noah_description's share directory path.
    pkg_noah_description = get_package_share_directory('noah_description')

    # Obtain urdf from xacro files.
    doc = xacro.process_file(os.path.join(pkg_noah_description, 'urdf', 'accurate_noah.urdf.xacro'), mappings={'radius': '0.9'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'publish_frequency': 30.0}

    # Robot state publisher
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                # namespace='noah',
                output='both',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )
    # Joint state publisher
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # namespace='noah',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration('jsp'))
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_noah_description, 'rviz', 'noah_description.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_argument,
        rsp_argument,
        jsp_argument,
        rviz,
        rsp,
        jsp,
    ])
