#!/bin/bash
echo "Creating testbench"
BINARY=/home/jst3si/temp/ros2_ws/install/rcl_executor_testbench/lib/rcl_executor_testbench/rcl_executor_testbench
RATE=1
MSG_SIZE=2
TOPOLOGY=F
PROCESS_TYPE=all
NUM_TOPICS=1
NUM_SUBS=10


echo script $BINARY $NUM_TOPICS $TOPOLOGY
python3 ./configure_test.py $BINARY $RATE $MSG_SIZE $TOPOLOGY $PROCESS_TYPE $NUM_TOPICS $NUM_SUBS

