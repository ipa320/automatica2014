#!/bin/bash

export ROS_MASTER_URI=http://scenario:11311
#. /opt/ros/hydro/setup.sh
. ~/hydro/devel/setup.sh

exec "$@"
