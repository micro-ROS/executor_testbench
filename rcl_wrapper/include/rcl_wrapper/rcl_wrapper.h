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

#ifndef RCL_EXECUTOR__RCL_WRAPPER_H_
#define RCL_EXECUTOR__RCL_WRAPPER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdarg.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>


/*! \file rcl_wrapper.h
    \brief rcl_wrapper provides a light-waight interface to the Ros Client Library.
*/

/// The init-object simplifies calls to rcl.
typedef struct {
  rcl_init_options_t init_options;
  rcl_context_t context;
  rcl_allocator_t * allocator;
  rcl_clock_t clock;
} rcl_init_wrapper_t;

/**
 *  Initialises RCL and creates an rcl-init object.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] init_obj a preallocated rcl_init_wrapper_t
 * \param[in] argc number of args of main
 * \param[in] argv array of arguments of main
 * \param[in] allocator allocator for allocating memory
 * \return `RCL_RET_OK` if RCL was initialized successfully
 * \return `RCL_RET_INVALID_ARGUMENT` if any null pointer as argument
 * \return `RCL_RET_ERROR` in case of failure
 */
rcl_ret_t
rcl_init_wrapper(
    rcl_init_wrapper_t * init_obj,
    int argc, 
    char const * const * argv, 
    rcl_allocator_t * allocator);

/**
 *  De-allocates the rcl-init object.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[inout] init_obj a preallocated rcl_init_wrapper_t
 * \return `RCL_RET_OK` if operation was successful
 * \return `RCL_RET_INVALID_ARGUMENT` if any null pointer as argument
 * \return `RCL_RET_ERROR` in case of failure
 */
rcl_ret_t
rcl_init_fini_wrapper(rcl_init_wrapper_t * init_obj);

/**
 *  Creates an RCL node.
 * 
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in this function and in RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] name the name of the node
 * \param[in] namespace the namespace of the node
 * \param[in] init_obj the rcl_init_wrapper_t object 
 * \return rcl_node_t if successful
 * \return NULL if an error occurred
 */
rcl_node_t *
rcl_create_node_wrapper(
    const char * name, 
    const char * namespace_, 
    rcl_init_wrapper_t * init_obj);

/**
 *  Deallocates memory of node.
 *  Result is a NULL pointer to `node`.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (de-allocates memory)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] init_obj the rcl_init_wrapper_t object
 * \param[inout] node the node to be de-allocated
 * \return `RCL_RET_OK` if successful
 * \return `RCL_RET_INVALID_ARGUMENT` if an argument is a null pointer
 * \return `RCL_RET_ERROR` in case of failure
 */
rcl_ret_t
rcl_node_fini_wrapper(
  rcl_init_wrapper_t * init_obj, 
  rcl_node_t * node);

/**
 *  Creates an rcl publisher.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in this function and RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] node the rcl node
 * \param[in] allocator the allocator to be used for memory allocation
 * \param[in] type_support the message data type
 * \param[in] topic_name the name of published topic
 * \return `rcl_publisher_t` if successful
 * \return `NULL` if an error occurred
 */
rcl_publisher_t *
rcl_create_publisher_wrapper(
  const rcl_node_t * node,
  rcl_allocator_t * allocator,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name);

/**
 *  Deallocates memory of rcl-publisher.
 *  Result is a NULL pointer to `publisher`.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (de-allocates memory)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] init_obj the rcl_init_wrapper_t object
 * \param[inout] publisher the rcl publisher to be de-allocated
 * \param[in] node the handle to the node used to create the publisher
 * \return `RCL_RET_OK` if successful
 * \return `RCL_RET_INVALID_ARGUMENT` if an argument is a null pointer
 * \return `RCL_RET_ERROR` in case of failure
 */
rcl_ret_t
rcl_publisher_fini_wrapper(
  rcl_init_wrapper_t * init_obj, 
  rcl_publisher_t * publisher, 
  rcl_node_t * node);

/**
 *  Creates an rcl subscription.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in this function and RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] node the rcl node
 * \param[in] allocator the allocator to be used for memory allocation
 * \param[in] type_support the message data type
 * \param[in] topic_name the name of subscribed topic
 * \return `rcl_subscription_t` if successful
 * \return `NULL` if an error occurred
 */
rcl_subscription_t *
rcl_create_subscription_wrapper(
  rcl_node_t * node,
  rcl_allocator_t * allocator,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name);

/**
 *  Deallocates memory of rcl-subscription.
 *  Result is a NULL pointer to `subscription`.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (de-allocates memory)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] init_obj the rcl_init_wrapper_t object
 * \param[inout] subscription the rcl subscription to be de-allocated
 * \param[in] node the handle to the node used to create the subscriber
 * \return `RCL_RET_OK` if successful
 * \return `RCL_RET_INVALID_ARGUMENT` if an argument is a null pointer
 * \return `RCL_RET_ERROR` in case of a failure
 */
rcl_ret_t
rcl_subscription_fini_wrapper(
  rcl_init_wrapper_t * init_obj, 
  rcl_subscription_t * subscription, 
  rcl_node_t * node);

/**
 *  Creates an rcl timer.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (in this function and RCL)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] init_obj the rcl_init_wrapper_t object
 * \param[in] timeout_ns the time out in nanoseconds of the timer
 * \param[in] callback the callback of the timer
 * \return `rcl_timer_t` if successful
 * \return `NULL` if an error occurred
 */
rcl_timer_t *
rcl_create_timer_wrapper(
  rcl_init_wrapper_t * init_obj,
  const uint64_t timeout_ns,
  const rcl_timer_callback_t callback);

/**
 *  Deallocates memory of an rcl-timer.
 *  Result is a NULL pointer to `timer`.
 *
 *  * <hr>
 * Attribute          | Adherence
 * ------------------ | -------------
 * Allocates Memory   | Yes (de-allocates memory)
 * Thread-Safe        | No
 * Uses Atomics       | No
 * Lock-Free          | Yes
 *
 * \param[in] init_obj the rcl_init_wrapper_t object
 * \param[inout] timer the timer to be de-allocated
 * \return `RCL_RET_OK` if successful
 * \return `RCL_RET_INVALID_ARGUMENT` if an argument is a null pointer
 * \return `RCL_RET_ERROR` in case of a failure
 */
rcl_ret_t
rcl_timer_fini_wrapper(
  rcl_init_wrapper_t * init_obj, 
  rcl_timer_t * timer);

/**
 * macro to print errors
 */
#define PRINT_RCL_ERROR(rclc, rcl) \
  do { \
    RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, \
      "[" #rclc "] error in " #rcl ": %s\n", rcutils_get_error_string().str); \
    rcl_reset_error(); \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif  // RCL_EXECUTOR__RCL_WRAPPER_H_
