FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-* \
    ros-humble-navigation2 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-domain-bridge \
    ros-humble-pcl-* \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install rosbags matplotlib numpy pytest psutil Pillow scipy pyyaml

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

CMD ["bash"]
