# https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/
# ROS distribution to use
ARG ROS_DISTRO=humble

#######################################
# Base Image for apt and pip packages #
#######################################
FROM ros:${ROS_DISTRO} as base
ENV ROS_DISTRO=${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=USERNAME
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /home/${USERNAME}/thesis-mppi-model-ident
WORKDIR /home/${USERNAME}/thesis-mppi-model-ident

# Install basic apt packages
COPY dependencies-apt.txt dependencies-apt.txt
RUN rm -f /etc/apt/apt.conf.d/docker-clean; \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN \
    --mount=type=cache,target=/var/cache/apt,mode=0777,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    xargs -a dependencies-apt.txt apt install -y --no-install-recommends

# Install newer libignition-math6 (6.15)
RUN \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt-get update && \
    apt-get install libignition-math6-dev -y

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install additional Python modules
COPY dependencies-pip.txt dependencies-pip.txt
RUN \
    --mount=type=cache,target=/root/.cache/pip,mode=0777,sharing=locked \
    pip3 install -r dependencies-pip.txt
# RUN pip3 install matplotlib transforms3d

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
    source /opt/ros/$ROS_DISTRO/setup.bash && \
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
    # adduser ${USERNAME} sudo && \
    echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    chown -R ${USERNAME} /home/${USERNAME} && \
    cp -r /root/.bashrc /home/${USERNAME}/

# Set the ownership of the overlay workspace to the new user
COPY ros_entrypoint /home/${USERNAME}/ros_entrypoint
RUN chown -R ${UID}:${GID} /home/${USERNAME}
RUN chmod +x /home/${USERNAME}/ros_entrypoint

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
RUN \
    curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-completion.bash \
            -o /home/${USERNAME}/git-completion.bash
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" \
    >> /home/${USERNAME}/.bashrc
RUN echo "source /home/$USERNAME/thesis-mppi-model-ident/workspace/install/setup.bash" \
    >> /home/${USERNAME}/.bashrc
RUN echo "source /home/${USERNAME}/thesis-mppi-model-ident/ros_entrypoint" \
    >> /home/${USERNAME}/.bashrc

RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models" \
    >> /home/${USERNAME}/.bashrc

ENTRYPOINT [ "/home/turtlewizard/ros_entrypoint" ]
CMD ["bash"]
