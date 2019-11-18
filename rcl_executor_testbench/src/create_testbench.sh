#!/bin/bash
echo "Creating testbench"
BINARY=/home/jst3si/temp/ros2_ws/install/rcl_executor_testbench/lib/rcl_executor_testbench/rcl_executor_testbench
GENERATOR_BIN=/home/jst3si/temp/ros2_ws/src/rcl_executor/rcl_executor_testbench/src/configure_test.py
RATE=1
MSG_SIZE=2
TOPOLOGY=F
PROCESS_TYPE=all
NUM_TOPICS=1
NUM_SUBS=10


echo script $BINARY $NUM_TOPICS $TOPOLOGY
python3 $GENERATOR_BIN $BINARY $RATE $MSG_SIZE $TOPOLOGY $PROCESS_TYPE $NUM_TOPICS $NUM_SUBS

