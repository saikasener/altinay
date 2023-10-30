#!/bin/bash
#source /home/rnd/.bashrc
ssh rnd@192.168.1.194 "source /home/rnd/.bashrc ; source /opt/ros/noetic/setup.bash ; source /home/rnd/moobot_ws/devel/setup.bash ; source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh ; rosnode kill /hector_mapping; rosnode kill /hector_trajectory_server ; rosnode kill /base_to_odom & echo \$!"