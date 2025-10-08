FROM osrf/ros:humble-desktop

# Install curl, certificates, starship

RUN apt-get update && apt-get install -y \
    curl \
    ca-certificates && \
    update-ca-certificates && \
    sh -c "$(curl -sS https://starship.rs/install.sh)" -- -y && \
    echo 'eval "$(starship init bash)"' >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*


# Install ROS packages, editors, fastfetch
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-demo-nodes-cpp \
    neovim \
    ripgrep \
    nano \
    tmux \
    usbutils \
    python3-serial \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    build-essential \
    software-properties-common && \
    add-apt-repository -y ppa:zhangsongcui3371/fastfetch && \
    apt-get update && apt-get install -y fastfetch && \
    rm -rf /var/lib/apt/lists/*

# Source ROS and run fastfetch on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "fastfetch || true" >> /root/.bashrc

# Copy Fastfetch config + ASCII
RUN mkdir -p /root/.config/fastfetch
COPY ./fastfetch/config.jsonc /root/.config/fastfetch/config.jsonc
COPY ./fastfetch/RAS-ascii-art.ans /root/.config/fastfetch/RAS-ascii-art.ans

#  Workspace setup 
WORKDIR /root/ros2_ws
RUN mkdir -p src


# Pre-source ROS when building workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build || true"

# Automatically source your workspace at container start
RUN echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

# Source workspace on start
WORKDIR /home
COPY . .

# Default shell
CMD ["bash"]
