FROM osrf/ros:humble-desktop-full

ARG COLCON_WS=/colcon_ws

COPY noah_description ${COLCON_WS}/src/noah_description
COPY noah_gazebo ${COLCON_WS}/src/noah_gazebo
COPY noah_integration_tests ${COLCON_WS}/src/noah_integration_tests
COPY noah_webots ${COLCON_WS}/src/noah_webots

# Install Webots
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget

RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -


RUN apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    webots \
    ros-humble-webots-ros2

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths ${COLCON_WS}/src --ignore-src -r -y && \
    colcon build --base-paths ${COLCON_WS}/src/ --build-base ${COLCON_WS}/build --install-base ${COLCON_WS}/install && \
    rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

COPY docker/entrypoint.bash entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]
WORKDIR ${COLCON_WS}
CMD ["/bin/bash"]
