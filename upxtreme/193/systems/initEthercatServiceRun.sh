source /opt/ros/noetic/setup.bash
source /home/rnd/moobot_ws/devel/setup.bash
source /home/rnd/moobot_ws/src/moobot_initialize/scripts/set_environment.sh
roslaunch moobot_control ethercat.launch &
