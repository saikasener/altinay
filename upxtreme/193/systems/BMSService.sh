#!/bin/bash
echo rnd | sudo -S chmod 666 /dev/ttyACM*
source /opt/ros/noetic/setup.bash
source /home/rnd/moobot_ws/devel/setup.bash
source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh
roslaunch moobot_bms moobot_bms.launch &
