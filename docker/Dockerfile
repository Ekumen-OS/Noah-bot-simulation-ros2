FROM osrf/ros:humble-desktop-full

ARG COLCON_WS=/colcon_ws

COPY noah_description ${COLCON_WS}/src/noah_description
COPY noah_gazebo ${COLCON_WS}/src/noah_gazebo
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths ${COLCON_WS}/src --ignore-src -r -y && \
    colcon build --base-paths ${COLCON_WS}/src/ --build-base ${COLCON_WS}/build --install-base ${COLCON_WS}/install

COPY docker/entrypoint.bash entrypoint.bash
ENTRYPOINT [ "/entrypoint.bash" ]

CMD ["/bin/bash"]
