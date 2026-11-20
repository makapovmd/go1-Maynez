#!/bin/bash
set -e

case "$*" in
    ping)
        printf '%s\n' '{"protocol_version":"1.0","supported_tasks":["T2"],"self_trained_rl":false,"preferred_speed_mps":null}'
        ;;
    T2)
        source /opt/ros/humble/setup.bash
        source /opt/unitree/local_setup.sh
        export PYTHONPATH="/opt/go1/python${PYTHONPATH:+:$PYTHONPATH}"
        exec python3 /opt/go1/safety_node.py --ros-args -p use_sim_time:=true
        ;;
    *)
        printf '%s\n' 'Usage: /entrypoint.sh ping|T2' >&2
        exit 2
        ;;
esac
