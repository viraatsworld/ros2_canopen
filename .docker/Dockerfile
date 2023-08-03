ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y \
        software-properties-common sudo \
        cmake clang curl can-utils net-tools \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

RUN apt-get update && apt-get install -y \
        libboost-all-dev libtool-bin ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-hardware-interface ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-realtime-tools ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-forward-command-controller \
&& rm -rf /var/lib/apt/lists/* \
&& apt-get clean

RUN mkdir -p /usr/local/bin/scripts
COPY .docker/entrypoint.sh /usr/local/bin/scripts/
RUN  chmod +x /usr/local/bin/scripts/*.sh

WORKDIR /home/colcon_ws
COPY . src

ENTRYPOINT [ "/usr/local/bin/scripts/entrypoint.sh" ]
