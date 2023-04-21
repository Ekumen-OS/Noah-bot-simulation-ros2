import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

# Obtain package
package_dir = get_package_share_directory('noah_webots')

def generate_launch_description():

    # The WebotsLauncher is used to start a Webots instance.
    # Arguments:
    # - `world` (str):              Path to the world to launch.
    # - `gui` (bool):               Whether to display GUI or not.
    # - `mode` (str):               Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool):   Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation. The Ros2Supervisor node is a special node interacting with the simulation. For example, it publishes the /clock topic of the simulation or permits to spawn robot from URDF files.
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'myWorld.wbt'),
        ros2_supervisor=True
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        # Starts the Ros2Supervisor node created with the WebotsLauncher
        webots._supervisor,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
