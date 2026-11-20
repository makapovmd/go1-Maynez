ARG ROS_BASE=ros:humble-ros-core-jammy@sha256:94acac3899448b2097ef8e925d8a9854ecd1fd7cb1690c3bbd7f7079f11ef258
FROM ${ROS_BASE} AS build

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake python3-colcon-ros python3-pip \
        ros-humble-ament-cmake ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY vendor/unitree_go src/unitree_go
RUN . /opt/ros/humble/setup.sh \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install \
        --install-base /opt/unitree --executor sequential \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

COPY requirements.lock .
RUN python3 -m pip install --no-cache-dir --no-deps --require-hashes \
    --target /opt/go1/python -r requirements.lock

FROM ${ROS_BASE}

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-numpy python3-zipp ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid 10001 participant \
    && useradd --uid 10001 --gid 10001 --create-home --no-log-init participant

COPY --chmod=755 entrypoint.sh /entrypoint.sh
COPY --from=build /opt/unitree /opt/unitree
COPY --from=build /opt/go1/python /opt/go1/python
COPY models/go1 /opt/go1/model
COPY t2/ /opt/go1/
COPY vendor/unitree_go/LICENSE /opt/go1/LICENSE.unitree_ros2
COPY vendor/LICENSE.simulator /opt/go1/LICENSE.simulator

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp MUJOCO_GL=disable OPENBLAS_NUM_THREADS=1
USER 10001:10001
WORKDIR /home/participant

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
