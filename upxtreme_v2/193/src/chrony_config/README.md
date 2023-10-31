We use chrony to syncronize network clocks between machines.

Instructions:

- Install chrony
	sudo apt-get install chrony

- For the host machine 
	# add the machine itself as a server
	# add these lines to your /etc/chrony/chrony.conf file in your host machine	
	server 127.0.0.1
	allow 127.0.0.1/8
	
	# allow the client machines in your host chrony.conf file with
	allow 192.168.10.101

- For the client machines
	# add the server IP or the hostname to follow up in the chrony.conf file
	# make sure you've added the hostname to /etc/hosts/
	server moobot_lattepanda iburst


- For both machines
	# Make sure these lines are uncommented to prevent your machines 
	# update the clock according to RTC or from the network
	#pool 2.debian.pool.ntp.org offline iburst
	#local stratum 10
	#hwclockfile /etc/adjtime
	#rtcsync
