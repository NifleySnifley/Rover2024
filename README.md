### On the development computer:
- `./build_robot_container.sh` creates the docker container (make sure to `rm -rf` the `build` and `install` directories if already build locally)
- `rosdep install --from-paths src -y --ignore-src` to install dependencies
- `colcon build` from root directory to build

### On the driver station:
- `sudo apt install ros-iron-spacenav` and then `ros2 run spacenav spacenav_node` for SpaceMouse control
- `sudo apt install ros-iron-joy-teleop`  and then `ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'` for teleop control with an Xbox controller
- For image viewing, use `rqt`, but first you need to install some compression plugins for smooth video: `sudo apt install ros-iron-compressed-image-transport ros-iron-image-transport-plugins`
- The best way to do graphs is with `plotjuggler`. `sudo apt install ros-iron-plotjuggler-ros` and to run it, `ros2 run plotjuggler plotjuggler`

### On the robot:
- Command for running docker container with robot code: `docker run -it --net host --ipc host --pid host --rm --privileged -v /dev/NavX_IMU:/dev/NavX_IMU nifleysnifley/rover2024 "/bin/bash"`
- To pull docker container: `docker pull nifleysnifley/rover2024`