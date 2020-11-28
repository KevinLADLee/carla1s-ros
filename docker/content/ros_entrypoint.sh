#!/bin/bash
set -e

# setup ros environment
source "/opt/carla-ros-bridge/devel/setup.bash"
exec "$@"