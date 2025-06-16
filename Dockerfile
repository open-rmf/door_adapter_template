FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip && \
        pip3 install flask-socketio websockets websocket-client requests && \
    rm -rf /var/lib/apt/lists/*

# Clone the repository
WORKDIR /door_adapter_template_ws/src
RUN git clone https://github.com/open-rmf/rmf_internal_msgs.git --branch main --single-branch --depth 1
COPY ./door_adapter_template door_adapter_template/

# Setup the workspace
WORKDIR /door_adapter_template_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
  && rm -rf /var/lib/apt/lists/*

# # Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# # Ensure the entrypoint script sources the ROS setup
RUN echo 'source /door_adapter_template_ws/install/setup.bash' >> /ros_entrypoint.sh

# # Ensure proper permissions for entrypoint
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

