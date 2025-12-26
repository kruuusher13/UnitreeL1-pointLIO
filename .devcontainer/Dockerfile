FROM --platform=linux/arm64 osrf/ros:humble-desktop-full

# Install Dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    ros-humble-foxglove-bridge \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-libpointmatcher \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep 
RUN rosdep update

# Environment sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc 


