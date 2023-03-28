#!/bin/bash
echo "" > lettuce_shaker.log
./examples/lettuce_shaker 172.16.0.2 10.0 0.2 10.0 2>&1 | tee -a lettuce_shaker.log
./examples/lettuce_shaker 172.16.0.2 10.0 0.5 10.0 2>&1 | tee -a lettuce_shaker.log
./examples/lettuce_shaker 172.16.0.2 10.0 1.0 10.0 2>&1 | tee -a lettuce_shaker.log
./examples/lettuce_shaker 172.16.0.2 10.0 2.0 10.0 2>&1 | tee -a lettuce_shaker.log
./examples/lettuce_shaker 172.16.0.2 10.0 3.0 10.0 2>&1 | tee -a lettuce_shaker.log
