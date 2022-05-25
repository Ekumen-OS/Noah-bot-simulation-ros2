# Noah-bot Gazebo Simulation - ROS2

## Overview

Gazebo simulation of the Noah-bot robot.

![noah-ros2](https://user-images.githubusercontent.com/53065142/170271725-93a1ae05-21e9-4c5c-8fef-5824da3ca23e.png)

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

## Official supported platform:
 - Ubuntu version: Focal Fossa 20.04
 - ROS Version: Foxy Fitzroy
 - Gazebo version: 11

_Note: Are you looking for the ROS1 simulation? See [here](https://github.com/Ekumen-OS/Noah-bot-simulation)._


## Installation using a Docker container

1. [Install docker](https://docs.docker.com/engine/install/ubuntu/)
2. Clone this repository.
3. Build the docker image.
   ```sh
   ./Noah-bot-simulation-ros2/docker/build
   ```
4. Run the script to run the docker container and mount the project. **IMPORTANT**: If the user does not own an nvidia gpu, delete the "--gpus all \" line from the script.
   ```sh
   ./Noah-bot-simulation-ros2/docker/run
   ```
5. Download dependencies for the packages
    ```sh
    cd /home/colcon_ws
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y
    ```
6. Build the project and source the workspace
    ```sh
    cd /home/colcon_ws
    colcon build
    source install/setup.bash
    source /usr/share/gazebo/setup.bash
    ```
7. It is ready to be used!. See [Run Simulation]
8. If more terminals are needed to be opened, in a new terminal run:
    ```sh
    docker exec -it noah_docker_ros2_process bash
    source /opt/ros/foxy/setup.bash
    source /home/colcon_ws/install/setup.bash
    # If running gazebo simulation here:
    source /usr/share/gazebo/setup.bash
    ```

## Non-containerized Installation

1. [Install ROS Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
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

## Organization

Summary of of the packages.
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

## Run Simulation!

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
