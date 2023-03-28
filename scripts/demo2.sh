#!/bin/bash
# ./examples/geometric_port_impedance_control 172.16.0.2 2>&1 | tee torques.log
./examples/geometric_port_impedance_control 172.16.0.2
# python3.10 ../../pyRobotPlot/main.py -f torques.log
