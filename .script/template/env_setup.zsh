#!/bin/zsh

: "${RMCS_PATH:=/workspaces/RMCS}"

# Default to local-only discovery for normal development, but allow
# callers to override this for remote DDS peers such as Isaac Sim.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RCUTILS_COLORIZED_OUTPUT=1

source /opt/ros/jazzy/setup.zsh

if [ -f "/rmcs_install/local_setup.zsh" ]; then
    source /rmcs_install/local_setup.zsh
elif [ -f "${RMCS_PATH}/rmcs_ws/install/local_setup.zsh" ]; then
    source "${RMCS_PATH}/rmcs_ws/install/local_setup.zsh"
fi

eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"

export RMCS_ROBOT_TYPE=""
export PATH="${PATH}:${RMCS_PATH}/.script"

fpath=("${RMCS_PATH}/.script/complete" $fpath)
autoload -Uz compinit
compinit
