#!/bin/bash
#source /home/rnd/.bashrc
ssh rnd@192.168.1.194 "source /home/rnd/.bashrc ; source /opt/ros/noetic/setup.bash ; source /home/rnd/moobot_ws/devel/setup.bash ; source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh ; rosnode kill /two_station_loop & echo \$!"