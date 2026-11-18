FROM ros:humble-ros-core-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd --gid 10001 participant \
    && useradd --uid 10001 --gid 10001 --create-home --no-log-init participant

COPY --chmod=755 entrypoint.sh /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
USER 10001:10001
WORKDIR /home/participant

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ping"]
