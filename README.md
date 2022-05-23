# noah-bot-simulation-ros2

## Build the packages

1. Clone the repository in your colcon workspace.
2. `rosdep install -y -r --from-paths src`
3. `colcon build --packages-up-to noah_gazebo`
4. `source install/setup.bash`

Note: If you haven't `source /usr/share/gazebo/setup.bash` please do it before launching the simulation.

## Organization

Organization of the packages.
### noah_description:
 - Contains Noah xacro files.
 - Simple launch file for:
   - Convert xacro to urdf
   - Run robot_state_publisher (Optional):
     - Publishes static TF.
     - Publishes robot description at /robot_description topic.
   - Run joint_state_publisher (Optional)
   - Run rviz2 (Optional)

### noah_gazebo:
 - Run gazebo simulation
   - World can be selected between `small_house.world` and `empty.world`.
 - Include `noah_description`'s launch file.
 - Spawn noah into gazebo.
 - Run rviz2 (optional)

## Try them !

Spawn noah in an empty world!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py
```

Use a different world!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py world:=small_house.world
```

Run rviz!

```sh
ros2 launch noah_gazebo noah_gazebo.launch.py rviz:=true
```

Teleop Noah!

In other terminal run:
```sh
ros2 run  teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=noah/cmd_vel
```


