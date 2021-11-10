#!/bin/bash

function log() {
  logger -s -p user.$1 ${@:2}
}

log info "base: Using workspace setup file /home/ubuntu/turtlebot2_catkin_ws/devel/setup.bash"
source /home/ubuntu/turtlebot2_catkin_ws/devel/setup.bash

log_path="/tmp"
if [[ ! -d $log_path ]]; then
  CREATED_LOGDIR=true
  trap 'CREATED_LOGDIR=false' ERR
    log warn "base: The log directory you specified \"$log_path\" does not exist. Attempting to create."
    mkdir -p $log_path 2>/dev/null
    chown ubuntu:ubuntu $log_path 2>/dev/null
    chmod ug+wr $log_path 2>/dev/null
  trap - ERR
  # if log_path could not be created, default to tmp
  if [[ $CREATED_LOGDIR == false ]]; then
    log warn "base: The log directory you specified \"$log_path\" cannot be created. Defaulting to \"/tmp\"!"
    log_path="/tmp"
  fi
fi

log info "base: Launching ROS_HOSTNAME=$ROS_HOSTNAME, ROS_IP=$ROS_IP, ROS_MASTER_URI=$ROS_MASTER_URI, ROS_LOG_DIR=$log_path"

# Punch it.
export ROS_HOME=$(echo ~ubuntu)/.ros
export ROS_LOG_DIR=$log_path





