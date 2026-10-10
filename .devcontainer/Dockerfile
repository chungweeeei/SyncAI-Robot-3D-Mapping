ARG ROS_DISTRO=humble
FROM ros:humble-ros-base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# - libboost-all-dev / libtbb-dev / libeigen3-dev: GTSAM's build and headers (same set as the
#   SyncAI-Robot-Workspace image, so it is built against the same libraries)
# - ros-*-pcl-conversions / pcl-ros: PCL (libpcl-dev) plus the PointCloud2 <-> pcl bridge
# - rmw-cyclonedds-cpp: the RMW devcontainer.json selects via RMW_IMPLEMENTATION; without it rcl
#   refuses to start
# - clang-format: the version CI's format job also pins (Ubuntu 22.04's clang-format 14)
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang-format \
        cmake \
        curl \
        gdb \
        git \
        libboost-all-dev \
        libeigen3-dev \
        libtbb-dev \
        python3-pip \
        python3-vcstool \
        sudo \
        tmux \
        vim \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# GTSAM 4.2.0, built from source into /usr/local exactly as SyncAI-Robot-Workspace's deps-builder
# stage does (there is no apt/PPA GTSAM on arm64). Flags follow the LIO-SAM recipe: system Eigen
# and no march-native to avoid Eigen-alignment crashes when mixed with PCL; TBB on; shared libs.
# Kept above everything that changes often: this layer takes a long time to build.
ARG GTSAM_VERSION=4.2.0
RUN git clone --branch ${GTSAM_VERSION} --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam && mkdir build && cd build \
    && cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_WITH_TBB=ON \
        -DBUILD_SHARED_LIBS=ON \
    && make -j"$(nproc)" && make install \
    && ldconfig \
    && rm -rf /tmp/gtsam

# OctoMap for build_octomap (the rosdep key `octomap`). Its own layer below GTSAM so adding apt
# packages here never invalidates the slow GTSAM build above.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-octomap \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# The repo is bind-mounted at /workspace/src/syncai_mapping on top of a named volume at
# /workspace. Create the mount point here so the volume is seeded with ros-owned directories;
# otherwise Docker creates src/ as root and nothing (e.g. a vcs-imported package) can be added
# next to this one.
RUN mkdir -p /workspace/src/syncai_mapping /workspace/build /workspace/install \
             /workspace/log \
    && chown -R ${USER_UID}:${USER_GID} /workspace /home/${USERNAME}

COPY setup_env.sh /opt/ros_setup.sh
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && printf '. /opt/ros_setup.sh\n' > /etc/profile.d/10-ros.sh \
    && printf '. /opt/ros_setup.sh\n' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

ARG INSTALL_CLAUDE_CODE=false
ENV CLAUDE_CONFIG_DIR=${HOME}/.claude
ENV PATH=${HOME}/.local/bin:${PATH}
RUN mkdir -p ${CLAUDE_CONFIG_DIR} \
    && if [ "${INSTALL_CLAUDE_CODE}" = "true" ]; then \
        curl -fsSL https://claude.ai/install.sh | bash; \
    fi

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
