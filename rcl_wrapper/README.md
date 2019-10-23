General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.

# The rcl_wrapper package

The package rcl_wrapper is a small [ROS 2](http://www.ros2.org/) package for providing a lightwaight abstraction on top of RCL to create nodes, publishers, subscribers and timers in a one-liner.

It it similar to rclc, however does not provide a spin() funktion for the node to receive messages. This is intentional because the executor concept shall be implemented a different package, e.g. rcl-executor.
Also, it does not create additional data structures, like rclc_node_t, rclc_publisher_t, rclc_subscriber_t etc. in the package rclc, but uses the rcl data structures instead. This light-waight and easy-to-use interface is intended for using ROS2 on micro-controllers without the rclcpp or rclpy. 

API:
- rcl_init_wrapper()
- rcl_init_fini_wrapper()
- rcl_create_node_wrapper()
- rcl_node_fini_wrapper()
- rcl_create_publisher_wrapper()
- rcl_publisher_fini_wrapper()
- rcl_create_subscription_wrapper()
- rcl_subscription_fini_wrapper()
- rcl_create_timer_wrapper()
- rcl_timer_fini_wrapper()


A complete code example with the `rcl-wrapper` and `rcl-executor` package is provided in the package `rcl_executor_examples`. See also the corresponding [README](../rcl_executor_examples/README.md)


