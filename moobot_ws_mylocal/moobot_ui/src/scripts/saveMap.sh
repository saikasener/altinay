#!/bin/bash
#source /home/rnd/.bashrc

ssh rnd@192.168.1.194 "source /home/rnd/.bashrc ; source /opt/ros/noetic/setup.bash ; source /home/rnd/moobot_ws/devel/setup.bash ; source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh ; rosrun map_server map_saver -f /home/rnd/moobot_ws/src/moobot_navigation/maps/test_map_saved & echo \$!"
ssh rnd@192.168.1.190 "source /home/rnd/.bashrc ; source /opt/ros/noetic/setup.bash ; source /home/rnd/moobot_ws/devel/setup.bash ; source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh ; rosrun map_server map_saver -f /home/rnd/moobot_ws/src/moobot_navigation/maps/test_map_saved & echo \$!"