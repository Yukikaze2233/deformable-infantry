#!/bin/bash

: "${RMCS_PATH:=/workspaces/RMCS}"

# Default to local-only discovery for normal development, but allow
# callers to override this for remote DDS peers such as Isaac Sim.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/jazzy/setup.bash

if [ -f "/rmcs_install/local_setup.bash" ]; then
    source /rmcs_install/local_setup.bash
elif [ -f "${RMCS_PATH}/rmcs_ws/install/local_setup.bash" ]; then
    source "${RMCS_PATH}/rmcs_ws/install/local_setup.bash"
fi

export RMCS_ROBOT_TYPE=""
