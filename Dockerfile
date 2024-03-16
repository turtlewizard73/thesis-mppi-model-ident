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

ARG UID=1000
ARG GID=${UID}

# Create new user and home directory
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

COPY ros_entrypoint.sh ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]
