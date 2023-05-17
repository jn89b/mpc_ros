#!/bin/bash

# Check if ROS 2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
  echo "Please source your ROS 2 environment first."
  exit 1
fi

# Check if the required argument is provided
if [ $# -lt 2 ]; then
  echo "Usage: $0 <output_file_name> <topic_name>"
  exit 1
fi

output_file="$1"
topic_name="$2"

# Start recording the ROS bag
ros2 bag record --output "$output_file" "$topic_name"


