General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The rcl_executor_examples package

rcl_executor_examples package provides an example for the rcl_exector.
- [example_publisher.cpp](example_publisher.cpp) contains a simple ROS node, publishing the topics cmd_vel cmd_hello
- [example_executor.c](example_executor.c) provides the example for a ROS node subscribing to these two topics `cmd_vel` and `cmd_hello`
- [rcl_wrapper_example.c](rcl_wrapper_example.c) is one self-contained example demonstrating the rcl-wrapper package to simplify creation of rcl handles. It creates one publisher and one subscriber within a rcl node. 

## Run the rcl_executor Example

**Step 1** ROS 2 Workspace Setup

Create two terminals with ROS 2 workspace. Assuming that the ROS 2 workspace resides in `/opt/ros/dashing` and the rcl_executor package is compiled in this workspace `~/rcl_executor_ws` then source the following setup scripts.
```C
~$ source /opt/ros/dashing/setup.bash
~$ source ~/rcl_executor_ws/install/local_setup.bash
```


**Step 2** Build

Build the package in the `rcl_executor_ws` and source the workspace (once).
```C
~/rcl_executor_ws/$ colcon_build
~$ source ~/rcl_executor_ws/install/local_setup.bash
```

**Step 2** Execute publisher

Run the example publisher.

```C
ros2 run rcl_executor_examples  example_publisher
```
 Then you should see something like this:


```C
[INFO] [rt_exectutor_test_publisher]: Publishing cmd_hello: 'Hello, world! 0'
[INFO] [rt_exectutor_test_publisher]: Publishing: cmd_vel '0.00'
[INFO] [rt_exectutor_test_publisher]: Publishing cmd_hello: 'Hello, world! 1'
[INFO] [rt_exectutor_test_publisher]: Publishing: cmd_vel '1.00'
[INFO] [rt_exectutor_test_publisher]: Publishing cmd_hello: 'Hello, world! 2'
[INFO] [rt_exectutor_test_publisher]: Publishing: cmd_vel '2.00'
[INFO] [rt_exectutor_test_publisher]: Publishing cmd_hello: 'Hello, world! 3'
[INFO] [rt_exectutor_test_publisher]: Publishing: cmd_vel '3.00'
```
**Step 3** Execute subscriber

Run the example subcriber

```C
ros2 run rcl_executor_examples  example_executor
```

Then you should see an output like the following:
```C
Created subscriber cmd_vel:
Created subscriber cmd_hello:
Created timer1 with timeout 100 ms.
Timer: time since last call 100170866
Timer: time since last call 99900511
Callback 'cmd_hello': I heard: Hello, world! 3
[#1] Callback 'cmd_vel': tv=3.000000 rv=3.000000
Callback 'cmd_hello': I heard: Hello, world! 4
Timer: time since last call 1555218878
[#2] Callback 'cmd_vel': tv=4.000000 rv=4.000000
Callback 'cmd_hello': I heard: Hello, world! 5
```
The first few messages were not received, because the subscriber was started before the publisher. This shows, that the messages are received by the ROS node, in which the subscriptions and timer are scheduled by the static LET Executor.

## Run the rcl_wrapper Example

**Step 1** ROS 2 Workspace Setup

Open a terminal with ROS 2 workspace. Assuming that the ROS 2 workspace resides in `/opt/ros/dashing` and the rcl_executor package is compiled in this workspace `~/rcl_executor_ws` then source the following setup scripts.
```C
~$ source /opt/ros/dashing/setup.bash
~$ source ~/rcl_executor_ws/install/local_setup.bash
```

**Step 2** Build

Build the package in the `rcl_executor_ws` and source the workspace (once).
```C
~/rcl_executor_ws/$ colcon_build
~$ source ~/rcl_executor_ws/install/local_setup.bash
```

**Step 2** Execute binary

Run the example:

```C
ros2 run rcl_executor_examples  example_rcl_wrapper
```
 Then you should see something like this:

```C
[INFO] []: Created a timer with period 500 ms.

Debug: number of DDS handles: 2
Published message aaaaaaaaaa
Callback: I heard: aaaaaaaaaa
Published message aaaaaaaaaa
Callback: I heard: aaaaaaaaaa
Published message aaaaaaaaaa
Callback: I heard: aaaaaaaaaa
Published message aaaaaaaaaa
Callback: I heard: aaaaaaaaaa
Published message aaaaaaaaaa
Callback: I heard: aaaaaaaaaa
```