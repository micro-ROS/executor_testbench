#!/bin/bash
#echo "Creating testbench"
PACKAGE=rcl_executor_testbench
BINARY=rcl_executor_testbench
GENERATOR_BIN=/home/jst3si/temp/ros2_ws/src/rcl_executor/rcl_executor_testbench/src/configure_test.py
RATE=1
MSG_SIZE=7
TOPOLOGY=F
PROCESS_TYPE=one
NUM_TOPICS=2
NUM_SUBS=3
NUM_PUBLISHED_MSG=3

#echo args: $GENERATOR_BIN $PACKAGE $BINARY $RATE $NUM_PUBLISHED_MSG $MSG_SIZE $TOPOLOGY $PROCESS_TYPE $NUM_TOPICS $NUM_SUBS
python3 $GENERATOR_BIN $PACKAGE $BINARY $RATE $NUM_PUBLISHED_MSG $NUM_RECEIVED_MSG $MSG_SIZE $TOPOLOGY $PROCESS_TYPE $NUM_TOPICS $NUM_SUBS

### executed following test cases

# default parameters:
#   RATE=1
#   MSG_SIZE=7
#   NUM_SUBS=3
#   NUM_PUBLISHED_MSG=3

#following states the parameters of each test case and the
# expected result
# look also at the printout eg:


# Node summary: Total number publishers : 1
#               Total number subscribers: 3
#
# topic_name for each publisher of node x
# topic_name for each subscriber of node x
# example for tc 1-1
#Debug: creating node_0
#  ... publishes topic_0
#Debug: creating node_1
# ... subscribes topic_0
#Debug: creating node_2
# ... subscribes topic_0
#Debug: creating node_3
# ... subscribes topic_0

# tc 1-1
# TOPOLOGY=A
# NUM_TOPICS=1
# PROCESS_TYPE=one
# => ...finished: publishers 1 received msg 9

# tc 1-2
# TOPOLOGY=A
# NUM_TOPICS=1
# PROCESS_TYPE=pub-sub
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 1-3
# TOPOLOGY=A
# NUM_TOPICS=1
# PROCESS_TYPE=all
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 0 received msg 3
# => ...finished: publishers 0 received msg 3
# => ...finished: publishers 0 received msg 3

# tc 2-1
# TOPOLOGY=B
# NUM_TOPICS=3
# PROCESS_TYPE=one
# => ...finished: publishers 3 received msg 9

# tc 2-2
# TOPOLOGY=B
# NUM_TOPICS=3
# PROCESS_TYPE=pub-sub
# => ...finished: publishers 3 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 2-3
# TOPOLOGY=B
# NUM_TOPICS=3
# PROCESS_TYPE=all
# => ...finished: publishers 3 received msg 0
# => ...finished: publishers 0 received msg 3
# => ...finished: publishers 0 received msg 3
# => ...finished: publishers 0 received msg 3

# tc 3-1
# TOPOLOGY=C
# NUM_TOPICS=1
# PROCESS_TYPE=one
# Node summary: Total number publishers : 3
#               Total number subscribers: 1

# => ...finished: publishers 3 received msg 9

# tc 3-2
# TOPOLOGY=C
# NUM_TOPICS=1
# PROCESS_TYPE=pub-sub
# Node summary: Total number publishers : 3
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 1

# => ...finished: publishers 3 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 3-3
# TOPOLOGY=C
# NUM_TOPICS=1
# PROCESS_TYPE=all
# Node summary: Total number publishers : 3
#               Total number subscribers: 0
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 1
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 4-1
# TOPOLOGY=D
# NUM_TOPICS=3
# PROCESS_TYPE=one
# Node summary: Total number publishers : 3
#               Total number subscribers: 3
# => ...finished: publishers 3 received msg 9

# tc 4-2
# TOPOLOGY=D
# NUM_TOPICS=3
# PROCESS_TYPE=pub-sub
# Node summary: Total number publishers : 3
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 3
# => ...finished: publishers 3 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 4-3
# TOPOLOGY=D
# NUM_TOPICS=3
# PROCESS_TYPE=all
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 3
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 0 received msg 9

# tc 5-1
# TOPOLOGY=E
# NUM_TOPICS=2
# PROCESS_TYPE=one
# Node summary: Total number publishers : 2
#               Total number subscribers: 6
# => ...finished: publishers 2 received msg 18

# tc 5-2
# TOPOLOGY=E
# NUM_TOPICS=3
# PROCESS_TYPE=pub-sub
# Node summary: Total number publishers : 2
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 6
# => ...finished: publishers 2 received msg 0
# => ...finished: publishers 0 received msg 18

# tc 5-3
# TOPOLOGY=E
# NUM_TOPICS=3
# PROCESS_TYPE=all
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 1
#               Total number subscribers: 0
# Node summary: Total number publishers : 0
#               Total number subscribers: 2
# Node summary: Total number publishers : 0
#               Total number subscribers: 2
# Node summary: Total number publishers : 0
#               Total number subscribers: 2
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 1 received msg 0
# => ...finished: publishers 0 received msg 6
# => ...finished: publishers 0 received msg 6
# => ...finished: publishers 0 received msg 6

# tc 6-1
# TOPOLOGY=F
# NUM_TOPICS=2
# PROCESS_TYPE=one
# Node summary: Total number publishers : 2
#               Total number subscribers: 6
# Debug: number of handles: 8
# Debug: creating node_0
#   ... publishes topic_0
#   ... publishes topic_1
#   ... subscribes topic_0
#   ... subscribes topic_1
#   ... subscribes topic_0
#   ... subscribes topic_1
#   ... subscribes topic_0
#   ... subscribes topic_1
# => ...finished: publishers 2 received msg 18
