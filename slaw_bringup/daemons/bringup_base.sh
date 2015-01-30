#!/bin/bash
source /opt/ros/hydro/setup.bash
source /home/youbot/ros/devel/setup.bash

#export ROS_PACKAGE_PATH=/home/youbot/ros/:$ROS_PACKAGE_PATH

DONE=0
while [ $DONE -lt 1 ] ; do
	if rosnode list
	then
		sleep 1
	        roslaunch --pid=/home/youbot/bringup_base.pid slaw_bringup bringup.launch > /home/youbot/base.log 2>&1
    		DONE=1
	else
	        sleep 1
		echo "no roscore started... waiting"
	fi
done
