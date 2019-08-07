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

/* test of library rcle_executor */
#include "rcl_executor/let_executor.h"
#include <unistd.h>
// ROS topics
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

/***************************** CALLBACKS *******************************************/


// callback for topic "cmd_hello"
void cmd_hello_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;


  if (msg == NULL) {
    printf("Callback: 'cmd_hello' msg NULL\n");
  } else {
    printf("Callback 'cmd_hello': I heard: %s\n", msg->data.data);
  }

  //sleep for 500ms
  usleep(500000);

}

// callback for topic "cmd_vel"
int numberMsgCmdVel = 0;
void cmd_vel_callback(const void * msgin)  // TwistConstPtr
{
  const geometry_msgs__msg__Twist * twist = (const geometry_msgs__msg__Twist *)msgin;
  numberMsgCmdVel++;
  // printf("cmd_vel received(#%d)\n", numberMsgCmdVel);

  if (twist != NULL) {
    printf("[#%d] Callback 'cmd_vel': tv=%f rv=%f \n", numberMsgCmdVel, (twist->linear.x),
      (twist->angular.z));
  } else {
    printf("Error callback commandVelCallback: Twist message is NULL.\n");
  }

  //sleep for 500ms

  usleep(500000);


}

// timer callback
void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  // Do timer work...
  // Optionally reconfigure, cancel, or reset the timer...
  if (timer != NULL) {
    printf("Timer: time since last call %d\n", (int) last_call_time);
  }
}

/******************** MAIN PROGRAM **************************************/
int main(int argc, const char * argv[])
{
  // rcl node initialization
  rcl_context_t context;                // global static var in rcl
  rcl_init_options_t init_options;      // global static var in rcl
  rcl_ret_t rc;

  init_options = rcl_get_zero_initialized_init_options();
  rc = rcl_init_options_init(&init_options, rcl_get_default_allocator());
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rt_executor, rcl_init_options_init);
    return -1;
  }

  context = rcl_get_zero_initialized_context();
  rc = rcl_init(argc, argv, &init_options, &context);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(init, rcl_init);
    return -1;
  }

  rc = rcl_init_options_fini(&init_options);   // necessary???

  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  rc = rcl_node_init(&node, "rcle_let_executor_test1_node", "", &context, &node_ops);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(create_node, rcl_node_init);
    return -1;
  } else {

    // create publisher
    /*
    const char* pose_topic = "robot_pose";

    rcl_publisher_t pub_odom        = rcl_get_zero_initialized_publisher();
    const rosidl_message_type_support_t * pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
    rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

    rc = rcl_publisher_init(
        &pub_odom,
        &node,
        pub_type_support,
        pose_topic,
        &pub_opt);

    if (rc != RCL_RET_OK) {
        PRINT_RCL_ERROR(create_publisher, rcl_publisher_init);
        printf("Failed to create publisher: %s.\n", pose_topic);
        return -1;
    } else {
        printf("Created publisher: %s\n", pose_topic);
    }
    */
    // create subscription
    const char * cmd_vel_topic_name = "cmd_vel";
    rcl_subscription_t sub_cmd_vel = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_ops1 = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t * sub_type_support1 = ROSIDL_GET_MSG_TYPE_SUPPORT(
      geometry_msgs, msg, Twist);

    geometry_msgs__msg__Twist msg1;

    rc = rcl_subscription_init(
      &sub_cmd_vel,
      &node,
      sub_type_support1,
      cmd_vel_topic_name,
      &subscription_ops1);

    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(create_subscription, rcl_subscription_init);
      printf("Failed to create subscriber: %s.\n", cmd_vel_topic_name);
      return -1;
    } else {
      printf("Created subscriber %s:\n", cmd_vel_topic_name);
    }

    // create subscription
    const char * cmd_hello_topic_name = "cmd_hello";
    rcl_subscription_t sub_cmd_hello = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_ops2 = rcl_subscription_get_default_options();
    const rosidl_message_type_support_t * sub_type_support2 = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,
        msg,
        String);
    std_msgs__msg__String msg2;

    rc = rcl_subscription_init(
      &sub_cmd_hello,
      &node,
      sub_type_support2,
      cmd_hello_topic_name,
      &subscription_ops2);

    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(create_subscription, rcl_subscription_init);
      printf("Failed to create subscriber: %s.\n", cmd_hello_topic_name);
      return -1;
    } else {
      printf("Created subscriber %s:\n", cmd_hello_topic_name);
    }

    // create timer with rcl
    rcl_clock_t clock;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rc = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(create_timer, rcl_clock_init);
      return -1;
    }
    rcl_timer_t timer1 = rcl_get_zero_initialized_timer();
    const unsigned int timer1_timeout = 100;
    rc = rcl_timer_init(&timer1, &clock, &context, RCL_MS_TO_NS(timer1_timeout),
        my_timer_callback, allocator);
    if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(create_timer, rcl_timer_init);
      return -1;
    } else {
      printf("Created timer1 with timeout %d ms.\n", timer1_timeout);
    }

    rcle_let_executor_t exe;
    rcle_let_executor_init(&exe, &context, 10, &allocator);

    rc = rcle_let_executor_add_subscription(&exe, &sub_cmd_vel, &msg1, &cmd_vel_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK) {PRINT_RCL_ERROR(rcle_executor, add_subscription);}

    rc = rcle_let_executor_add_subscription(&exe, &sub_cmd_hello, &msg2, &cmd_hello_callback,
        ON_NEW_DATA);
    if (rc != RCL_RET_OK) {PRINT_RCL_ERROR(rcle_executor, add_subscription);}

    rcle_let_executor_add_timer(&exe, &timer1);
    if (rc != RCL_RET_OK) {PRINT_RCL_ERROR(rcle_executor, add_timer);}

    //set time_out for wait_set in nanoseconds 100ms = 100 000 000 ns
    rc = rcle_let_executor_set_timeout(&exe, 100000000);
    if (rc != RCL_RET_OK) {PRINT_RCL_ERROR(rcle_executor, set_timeout);}

    rcle_let_executor_spin(&exe);
    rcle_let_executor_fini(&exe);

    // clean up and free memory
    rc = rcl_subscription_fini(&sub_cmd_vel, &node);
    rc = rcl_subscription_fini(&sub_cmd_hello, &node);
    rc = rcl_timer_fini(&timer1);
    rc = rcl_node_fini(&node);
  }
  return 0;
}
