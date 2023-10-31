#!/bin/bash
# ----------------------------------------------------------------------------
# EcMasterDemoPythonInteractive.sh
# ----------------------------------------------------------------------------
# Start EC-Master demo for Python in interactive mode
# ----------------------------------------------------------------------------
# --mode 1
#     Demo runs in master mode. Supported modes:
#     1 = Master
#     2 = RasClient
#     3 = MbxGateway
#     4 = Simulator
# -b 4000
#     Cycle time in usec
# -t 0
#     Demo runs endless
# -v 3
#     Verbosity level is 3 (0..7)
# --link "sockraw eth0 1" 
#     Link layer with settings
#     Please replace eth0 with the name
#     of the network adapter connected to the EtherCAT network
# -f ENI.xml
#     Please provide the ENI file matching the connected etherCAT network
# --sp
#     Start RAS server for EC-Engineer connection
# ----------------------------------------------------------------------------
export PYTHONPATH=../../Sources/EcWrapperPython:$PYTHONPATH
export PATH=./x64:./x86:$PATH
export LD_LIBRARY_PATH=./x64:./x86:$LD_LIBRARY_PATH
python3 -i ../../Examples/EcMasterDemoPython/EcDemoAppInteractive.py --mode 1 -b 4000 -t 0 -v 3 --link "sockraw eth0 1" -f ENI.xml
