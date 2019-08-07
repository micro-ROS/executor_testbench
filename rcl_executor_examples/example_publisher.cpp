// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/micro-ROS/rcl_executor.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rt_exectutor_test_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("cmd_hello",
      rclcpp::SystemDefaultsQoS());
  auto message = std::make_shared<std_msgs::msg::String>();
  auto publish_count = 0;

  auto publisher2 = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
      rclcpp::SystemDefaultsQoS());
  auto message2 = std::make_shared<geometry_msgs::msg::Twist>();


  rclcpp::WallRate loop_rate(500ms);

  while (rclcpp::ok()) {
    // topic cmd_hello
    message->data = "Hello, world! " + std::to_string(publish_count);
    RCLCPP_INFO(node->get_logger(), "Publishing cmd_hello: '%s'", message->data.c_str());
    publisher->publish(*message);

    // topic cmd_vel
    message2->linear.x = publish_count;
    message2->angular.z = publish_count;
    RCLCPP_INFO(node->get_logger(), "Publishing: cmd_vel '%.2f'", message2->linear.x);
    publisher2->publish(*message2);

    // update counter
    publish_count++;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
