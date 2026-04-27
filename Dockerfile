FROM osrf/ros:humble-desktop-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# =========================
# Dependências base ROS + SLAM
# =========================
RUN apt update && apt install -y \
    git cmake build-essential pkg-config \
    libeigen3-dev libboost-all-dev \
    libglew-dev libgl1-mesa-dev \
    libopencv-dev \
    python3-colcon-common-extensions \
    python3-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-message-filters \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# =========================
# Pangolin (versão estável SEM python bindings)
# =========================
RUN git clone https://github.com/stevenlovegrove/Pangolin.git /opt/Pangolin && \
    cd /opt/Pangolin && \
    git checkout ad8b5f8 && \
    mkdir build && cd build && \
    cmake .. \
      -DBUILD_PANGOLIN_PYTHON=OFF \
      -DBUILD_PYTHON=OFF \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TOOLS=OFF \
      -DBUILD_TESTS=OFF \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_DISABLE_FIND_PACKAGE_pybind11=ON \
      -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    ldconfig
# garante linker enxergar libpangolin
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/pangolin.conf && ldconfig

# =========================
# ORB-SLAM3
# =========================
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3 && \
    cd /opt/ORB_SLAM3 && \
    chmod +x build.sh && \
    ./build.sh

# =========================
# Workspace ROS2
# =========================
RUN mkdir -p /ws/src
WORKDIR /ws

RUN source /opt/ros/humble/setup.bash && colcon build
RUN apt-get update && apt-get install nano

# =========================
# ENV ROS automático
# =========================
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
