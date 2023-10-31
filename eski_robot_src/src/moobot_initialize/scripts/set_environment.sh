#!/bin/bash
#
# Network Info
# SSID: AGV_Network
# WPA Passphrase: 89954164
#
 remote_device_ip="192.168.1.190"
 moobot_1="192.168.1.198"
 moobot_2="192.168.1.193"
 current_device_ip="$(ip -4 addr | grep enp* | grep -oP '(?<=inet\s)\d+(\.\d+){3}')"
#
# Export the ROS Variables for Initialization
 export ROS_MASTER_URI="http://$moobot_2:11311"
 export ROS_HOSTNAME="192.168.1.193"
#

