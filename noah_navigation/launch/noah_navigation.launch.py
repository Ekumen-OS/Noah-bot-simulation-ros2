import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    use_slam = LaunchConfiguration("slam", default="False")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("noah_navigation"), "maps", "small_house.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("noah_navigation"), "params", "noah_navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("noah_navigation"), "rviz", "noah_navigation.rviz")


    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    return LaunchDescription(
        [
            DeclareLaunchArgument("slam", default_value=use_slam, description="Whether run a SLAM"),
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock if true"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_dir, "use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir, "slam": use_slam}.items(),
            ),
        ]
    )
