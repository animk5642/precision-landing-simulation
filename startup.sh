#!/usr/bin/env bash
set -e

gz sim -r /mnt/data/Documents/precision-landing-simulation/config/my_custom_world.sdf &
GZ_PID=$!

cleanup() {
    kill "$GZ_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 5

gz topic -t /gimbal/cmd_pitch \
  -m gz.msgs.Double -p 'data: 1.5'

wait "$GZ_PID"

