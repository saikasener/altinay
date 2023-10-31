#!/bin/bash
source /home/rnd/.bashrc
source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh
source /opt/ros/noetic/setup.bash
source /home/rnd/moobot_ws/devel/setup.bash
roslaunch moobot_bringup moobot_core_new.launch &
