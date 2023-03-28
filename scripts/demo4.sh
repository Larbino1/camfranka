#!/bin/bash
# ./examples/port_impedance_control 172.16.0.2 >&1 | tee port_err.log
./examples/port_impedance_control 172.16.0.2
# python3.10 ../../pyRobotPlot/main.py --port-error -f ./port_err.log
