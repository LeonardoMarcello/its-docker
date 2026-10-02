FROM ros:humble-ros-core

ARG USER=its
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    locales \
    sudo \
    wget \
    curl \
    libeigen3-dev \
    libblas-dev \
    liblapack-dev \
    libboost-all-dev \
    libgl1-mesa-dev \
    liboctomap-dev \
    xvfb \
    freeglut3-dev \
    libassimp-dev \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-eigenpy \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update || true

# Clone, compile, and install Coal, then clean up to save image space
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /tmp && \
    git clone --recursive https://github.com/coal-library/coal.git && \
    cd coal && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j\$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/coal"

RUN rosdep update || true

# create a non-root user to match host UID/GID
RUN groupadd -g ${GID} ${USER} || true && \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USER} || true && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER}

# copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown root:root /entrypoint.sh


USER ${USER}
ENV HOME=/home/${USER}
WORKDIR ${HOME}

# Source ROS 2 automatically for the 'its' user
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/its/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/its/.bashrc


ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
