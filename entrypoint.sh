#!/bin/sh
set -eu

if [ "$#" -eq 1 ] && [ "$1" = ping ]; then
    printf '%s\n' '{"protocol_version":"1.0","supported_tasks":[],"self_trained_rl":false,"preferred_speed_mps":null}'
    exit 0
fi

printf '%s\n' 'Usage: /entrypoint.sh ping' >&2
exit 2
