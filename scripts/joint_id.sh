#!/bin/bash
echo "" > joint_id.log
./examples/joint_identification 172.16.0.2 7 0.75 0.1 10.0 2>&1 | tee -a joint_id.log
./examples/joint_identification 172.16.0.2 7 0.75 0.2  5.0 2>&1 | tee -a joint_id.log
./examples/joint_identification 172.16.0.2 7 0.5  0.5  4.0 2>&1 | tee -a joint_id.log
./examples/joint_identification 172.16.0.2 7 0.5  1.0  3.0 2>&1 | tee -a joint_id.log
#./examples/joint_identification 172.16.0.2 7 0.1  2.0  .5 2>&1 | tee joint_id.log
#./examples/joint_identification 172.16.0.2 7 0.2  4.0  3.0 2>&1 | tee joint_id.log
