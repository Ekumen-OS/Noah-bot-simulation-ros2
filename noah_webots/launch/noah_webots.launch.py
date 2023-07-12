# BSD 3-Clause License

# Copyright (c) 2023, Ekumen Inc.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import launch
import os
import pathlib
import xacro

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.webots_launcher import WebotsLauncher

# Obtain packages
curr_pkg_dir = get_package_share_directory('noah_webots')
noah_pkg_dir = get_package_share_directory('noah_description')

def generate_launch_description():

    noah_xacro_path = os.path.join(noah_pkg_dir, 'urdf', 'accurate_noah.urdf.xacro')
    noah_description = xacro.process_file(noah_xacro_path).toprettyxml(indent='    ')

    spawn_URDF_noah = URDFSpawner(
        name='accurate_noah',
        robot_description=noah_description,
        translation='0 1 0.022',
        rotation=' 0 0 1 0',
    )

    # The WebotsLauncher is used to start a Webots instance.
    # Arguments:
    # - `world` (str):              Path to the world to launch.
    # - `gui` (bool):               Whether to display GUI or not.
    # - `mode` (str):               Can be `pause`, `realtime`, or `fast`.
    # - `ros2_supervisor` (bool):   Spawn the `Ros2Supervisor` custom node that communicates with a Supervisor robot in the simulation.
    webots = WebotsLauncher(
        world=os.path.join(curr_pkg_dir, 'worlds', 'myWorld.wbt'),
        ros2_supervisor=True
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        # Starts the Ros2Supervisor node created with the WebotsLauncher
        webots._supervisor,
        # Spawn Noah's URDF
        spawn_URDF_noah,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
