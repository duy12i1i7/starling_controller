ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:${VERSION}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libboost-system-dev \
        git \
        wget \
        python3-pip \
        python3-scipy \
        libgeographic-dev \
        geographiclib-tools \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

## Ensure geographiclib is properly installed with FindGeographicLib available
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && bash install_geographiclib_datasets.sh \
    && ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/


# Copies all the nodes into the container
COPY starling_controller /ros_ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build\
    && rm -r build

COPY starling_controller/run.sh /ros_ws
CMD ["./run.sh"]