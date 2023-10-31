#!/bin/bash

# Start navigation
roslaunch /home/agv/moobot_ws/src/moobot_navigation/launch/moobot_navigation.launch &
# Determine the Arduino Serial Port
 arduino_serial_port=$(ls /dev | grep ACM)
#
echo "Checking the connected devices..."
echo
# Arduino Port Control
 if [ "$arduino_serial_port" != "" ]; then
 	echo "- Arduino is connected."
	echo
	#
	# Start the roscore
	echo "Starting the roscore..."
 	echo
 	sleep 5 &&
	rosrun rosserial_python serial_node.py /dev/ttyACM0
	killall rosmaster > /dev/null 2>&1
 else
	echo -e "\033[31mArduino is not connected." 1>&2
 fi

