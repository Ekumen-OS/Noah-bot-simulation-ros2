# Noah-bot Gazebo Simulation - ROS2

## Overview

Gazebo simulation for the Noah-bot robot.

![noah-ros2](https://user-images.githubusercontent.com/53065142/170271725-93a1ae05-21e9-4c5c-8fef-5824da3ca23e.png)

### URDF
It contains the URDF model of the robot with accurate visual meshes.
### Sensors
 - Kinect Camera. Ros topics:
    - /noah/kinect/camera_info
    - /noah/kinect/depth/camera_info
    - /noah/kinect/depth/image_raw
    - /noah/kinect/image_raw
    - /noah/kinect/points
 - Laser Scanner. Ros topics:
    - /noah/laser_scan

### Diff drive controller
 Ros topics:
 - /noah/odom
 - /noah/cmd_vel

## Officially supported platform:
 - Ubuntu version: Jammy Jellyfish (22.04)
 - ROS Version: Humble
 - Gazebo version: 11

_Note: Are you looking for the ROS1 simulation? See [here](https://github.com/Ekumen-OS/Noah-bot-simulation)._


## Repository organization

Summary of of the packages in the repository.
### noah_description:
 - Holds Noah xacro files for URDF definition.
 - Simple launch file for:
   - Convert xacro to urdf
   - Run `robot_state_publisher` (Optional):
     - Publishes static `TF`.
     - Publishes robot description at `/noah/robot_description` topic.
   - Run `joint_state_publisher` (Optional)
   - Run `rviz2` (Optional)

### noah_gazebo:
 - Holds three gazebo worlds to spawn Noah in: `empty.world`, `test.world` and `small_house.world`.
 - Launch file for:
   - Run gazebo using an available world.
   - Includes `noah_description` launch file.Include `noah_description`'s launch file.
   - Spawn Noah
   - Run rviz2 (optional)


## Using docker

### Building the code

1. [Install docker](https://docs.docker.com/engine/install/ubuntu/)
2. If using nvidia, install [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)
3. Clone this repository.
4. Build the docker image running:
   ```sh
   DOCKER_BUILDKIT=1 docker build -t noahbot:humble -f docker/Dockerfile .
   ```
   After this, the image tagged as `noahbot:humble` will be ready to run the Noahbot simulation.

### Running the simulation

Run:
```sh
./docker/run ros2 launch noah_gazebo noah_gazebo.launch.py
```
To start the simulation.
**IMPORTANT**: If you do not want to use the host GPUs, delete the "--gpus all \" line from the script.

### Quickly test changes without rebuilding the container

If you want to test code changes quickly without rebuilding the container, you can use the `develop` script:

```sh
$ ./docker/develop
```

Which will open a shell inside the container, mounting the local copy of the ROS packages.
You can quickly re-build the code with:

```sh
$ cd /colcon_ws
$ colcon build
```

The `develop` script will also run the containers with host networking, so if you open multiple ones
you can easily send ROS messages across them.

## Non-containerized Installation

1. [Install ROS humble Fitzroy](https://docs.ros.org/en/humble/Installation.html)
2. [Install Gazebo 11](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
3. Clone this repository in your `colcon` workspace.
    ```
    ├── colcon_ws
        └── src
            └── Noah-bot-simulation-ros2
    ```
4. `rosdep install --from-paths src --ignore-src -r -y`
5. Build the project and source the workspace
    ```sh
    colcon build
    source install/setup.bash
    source /usr/share/gazebo/setup.bash
    ```
6. It is ready to be used!. See [Run Simulation](README.md#run-simulation)!


### Run Simulation!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py
```

Use a different world!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py world:=small_house.world
```

Use the `rviz` argument and run `rviz2` along the simulation!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py rviz:=true
```

Try Teleoperating Noah!

Once simulation is running, in other terminal run:
```sh
ros2 run  teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=noah/cmd_vel
```
