General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The rcl_executor_testbench package
This package provides a script to generate multiple test cases for executors. You can configure 
- number of topics
- rate
- message length
- topology type
- communication type
- number of subscriptions

In a first step, command-line arguments are generated according the the configuration. In a second step, the rcl-executor is instantiated with the corresponding number of nodes, publishers, timers and subscriptions. 
