# Use ROS 2 Jazzy as the base image
FROM ros:jazzy

# Set up the environment
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    python3-pyqt6 \
    python3-sympy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /root/multi_robots_ws
COPY ../. /root/multi_robots_ws

#Manually copy symbolic links as they somteimes do not work in docker
RUN cp /root/multi_robots_ws/pioneer_base/lib/my_ros_module.py /root/multi_robots_ws/pioneer_base/teleop_core/teleop_core/my_ros_module.py
RUN cp /root/multi_robots_ws/pioneer_base/lib/my_ros_module.py /root/multi_robots_ws/pioneer_base/sensor_receiver/sensor_receiver/my_ros_module.py
RUN cp /root/multi_robots_ws/pioneer_base/lib/my_ros_module.py /root/multi_robots_ws/pioneer_base/mode_select/mode_select/my_ros_module.py
RUN cp /root/multi_robots_ws/pioneer_base/lib/my_ros_module.py /root/multi_robots_ws/pioneer_base/controller/controller/my_ros_module.py
RUN cp /root/multi_robots_ws/pioneer_base/lib/my_ros_module.py /root/multi_robots_ws/pioneer_base/auto_nav_core/auto_nav_core/my_ros_module.py
RUN cp /root/multi_robots_ws/sim/lib/my_ros_module.py /root/multi_robots_ws/sim/fake_rover_state_controller/fake_rover_state_controller/my_ros_module.py
RUN cp /root/multi_robots_ws/sim/lib/my_ros_module.py /root/multi_robots_ws/sim/high_level_controller/high_level_controller/my_ros_module.py


# Source the workspace on container start
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "colcon build" >> ~/.bashrc
RUN echo "source /root/multi_robots_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
