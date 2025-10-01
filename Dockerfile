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

WORKDIR /home
COPY . .

