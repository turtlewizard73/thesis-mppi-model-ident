# syntax = docker/dockerfile:1.3
# https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
# ROS distribution to use
ARG ROS_DISTRO=humble

#######################################
# Base Image for apt and pip packages #
#######################################
FROM ros:${ROS_DISTRO} as base
ENV ROS_DISTRO=${ROS_DISTRO}
ARG USERNAME=USERNAME
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /home/${USERNAME}
WORKDIR /home/${USERNAME}/thesis-mppi-model-ident

# Install basic apt packages
COPY dependencies-apt.txt dependencies-apt.txt
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt-get update && \
    xargs -a dependencies-apt.txt apt install -y --no-install-recommends

# Install additional Python modules
COPY dependencies-pip.txt dependencies-pip.txt
RUN \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    pip3 install -r dependencies-pip.txt
# RUN pip3 install matplotlib transforms3d

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

##########################################
# Underlay Image for rosdep dependencies #
##########################################
FROM base as underlay

COPY workspace workspace
WORKDIR /home/${USERNAME}/thesis-mppi-model-ident/workspace
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.ros/rosdep,mode=0777,sharing=locked \
    # rm -f /etc/apt/apt.conf.d/docker-clean && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro=$ROS_DISTRO

#####################################
# Overlay Image for everything else #
#####################################
FROM underlay AS overlay
WORKDIR /home/${USERNAME}/thesis-mppi-model-ident

# Install dev apt packages - these pacakges are used for custom ros2 pkgs
COPY dependencies-apt_dev.txt dependencies-apt_dev.txt
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt-get update && \
    xargs -a dependencies-apt_dev.txt apt install -y --no-install-recommends

# Install dev Python modules - these pacakges are used for custom ros2 pkgs
COPY dependencies-pip_dev.txt dependencies-pip_dev.txt
RUN \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    pip3 install -r dependencies-pip_dev.txt

# # build the workspace
# WORKDIR /home/${USERNAME}/thesis-mppi-model-ident/workspace
# # Check if "nav2_humble_cache" folder contains "build" and "install" folders
# RUN if [ -d "../nav2_humble_cache/build" ] && [ -d "../nav2_humble_cache/install" ]; then \
#         echo "NAV2 already cached, copying and skipping build." && \
#         cp -r ../nav2_humble_cache/build ./ && \
#         cp -r ../nav2_humble_cache/install ./ ; \
#     else \
#         echo "nav2_humble_cache folder does not contain build and install folders, building..." && \
#         source /opt/ros/$ROS_DISTRO/setup.bash && \
#         colcon build --symlink-install && \
#         cp -r ./build ../nav2_humble_cache/build && \
#         cp -r ./install ../nav2_humble_cache/install ; \
#     fi

# Create new user and home directory
ARG UID=1000
ARG GID=${UID}

RUN \
    groupadd --gid $GID $USERNAME && \
    useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} && \
    adduser ${USERNAME} sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    chown -R ${UID}:${GID} /home/${USERNAME}

# Set the ownership of the overlay workspace to the new user
RUN chown -R ${UID}:${GID} /home/${USERNAME}/thesis-mppi-model-ident

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${USERNAME}/.bashrc
RUN echo "source /home/$USERNAME/thesis-mppi-model-ident/workspace/install/setup.bash" \
    >> /home/${USERNAME}/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

COPY ros_entrypoint.sh ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]
