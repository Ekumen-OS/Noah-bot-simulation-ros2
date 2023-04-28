## Noah Webots package

This package contains all the necessary resources to launch a Webots simulation with the capacity to import and spawn a Noah robot from the `noah_description` package.

### Running the simulation

Once the package has been built with `colcon build` and the setup file sourced, the simulation can be launched by running:

`ros2 launch noah_webots noah_webots.launch.py`

This starts Webots and loads the world `myWorld` stored in `/worlds`, along with a Supervisor Node provided by `webots_ros2` called `Ros2Supervisor`. This is a special Supervisor node that interacts with the simulation, allowing different features such as publishing the `/clock` topic, or spawning a robot from a URDF file. This node will be useful when importing an URDF robot in a running simulation. 
