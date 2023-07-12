FROM osrf/ros:humble-desktop-full

ARG COLCON_WS=/colcon_ws
ARG WEBOTS_ROS2_WS=/ros2_ws

COPY noah_description ${COLCON_WS}/src/noah_description
COPY noah_gazebo ${COLCON_WS}/src/noah_gazebo
COPY noah_integration_tests ${COLCON_WS}/src/noah_integration_tests
COPY noah_webots ${COLCON_WS}/src/noah_webots

# Install Webots
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget

RUN sudo wget -q https://cyberbotics.com/Cyberbotics.asc -O /etc/apt/keyrings/Cyberbotics.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    webots

# Install the 2023.1.1 release of webots_ros2
RUN git clone --recurse-submodules --branch 2023.1.1 https://github.com/cyberbotics/webots_ros2.git ${WEBOTS_ROS2_WS}/src/webots_ros2

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths ${COLCON_WS}/src ${WEBOTS_ROS2_WS}/src --ignore-src -r -y && \
    colcon build --base-paths ${COLCON_WS}/src/ --build-base ${COLCON_WS}/build --install-base ${COLCON_WS}/install && \
    colcon build --base-paths ${WEBOTS_ROS2_WS}/src/ --build-base ${WEBOTS_ROS2_WS}/build --install-base ${WEBOTS_ROS2_WS}/install && \
    rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

COPY docker/entrypoint.bash entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]
WORKDIR ${COLCON_WS}
CMD ["/bin/bash"]
