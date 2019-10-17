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

#include "rcl_executor/let_executor.h"
#include <std_msgs/msg/string.h>
// #include <unistd.h> // for usleep()

// global data structures

static rcl_node_t * nodes;
static rcl_publisher_t * pubs;
static std_msgs__msg__String pub_msg;

static rcl_subscription_t * subs;
static std_msgs__msg__String * subs_msg;


/*** rcl convenience functions ***/

typedef struct {
  rcl_init_options_t init_options;
  rcl_context_t context;
  rcl_allocator_t * allocator;
  rcl_clock_t clock;
} rcl_init_wrapper_t;

/* assignes modifies init_obj */
rcl_ret_t
rcl_init_wrapper(
    rcl_init_wrapper_t * init_obj,
    int argc, 
    char const * const * argv, 
    rcl_allocator_t * allocator)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
    init_obj, "init_obj is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc = RCL_RET_OK;

  init_obj->init_options = rcl_get_zero_initialized_init_options();
  rc = rcl_init_options_init(&init_obj->init_options, (*allocator) );
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_init_wrapper, rcl_init_options_init);
    return rc;
  }

  init_obj->context = rcl_get_zero_initialized_context();
  rc = rcl_init(argc, argv, &init_obj->init_options, &init_obj->context);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_init_wrapper, rcl_init);
    return rc;
  }
  init_obj->allocator = allocator;
  return rc;
}

rcl_node_t *
rcl_create_node_wrapper(
    const char * name, 
    const char * namespace_, 
    rcl_init_wrapper_t * init_obj)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  name, "name is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  namespace_, "namespace_ is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  init_obj, "init_obj is a null pointer", return NULL);

  rcl_ret_t rc;
  rcl_node_t * node = init_obj->allocator->allocate( sizeof(rcl_node_t), init_obj->allocator->state);
  if ( node != NULL){
    (*node) = rcl_get_zero_initialized_node();
    // node_ops is copied to ...->impl->node-options, therefore temporary scope sufficient
    rcl_node_options_t node_ops = rcl_node_get_default_options();  
    rc = rcl_node_init(node, name, namespace_, 
      &init_obj->context, &node_ops);
    if (rc != RCL_RET_OK) {
      init_obj->allocator->deallocate( node, init_obj->allocator->state );
      PRINT_RCL_ERROR(rcl_create_node_wrapper, rcl_node_init);
      return NULL;
    } 
  }
  return node;
}

rcl_ret_t
rcl_node_fini_wrapper(rcl_init_wrapper_t * init_obj, rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  // clean-up rcl_node_t
  rcl_ret_t rc = rcl_node_fini(node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_node_fini_wrapper, rcl_node_fini);
  }

  // de-allocate node itself
  init_obj->allocator->deallocate(node, init_obj->allocator->state);

  return rc;
}

// todo where is type_support free-ed again?

rcl_publisher_t *
rcl_create_publisher_wrapper(
  const rcl_node_t * node,
  rcl_allocator_t * allocator,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  node, "node is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  allocator, "allocator is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  type_support, "type_support is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  topic_name, "topic_name is a null pointer", return NULL);

  rcl_publisher_t * pub =  allocator->allocate( 
    sizeof(rcl_publisher_t), allocator->state);
  (*pub) = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();
  rcl_ret_t rc = rcl_publisher_init(
    pub,
    node,
    type_support,
    topic_name,
    &pub_opt);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_create_publisher_wrapper, rcl_publisher_init);
    allocator->deallocate( pub, allocator->state);
    return NULL;
  }
  return pub;
}

rcl_ret_t
rcl_publisher_fini_wrapper(rcl_init_wrapper_t * init_obj, rcl_publisher_t * publisher, rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  init_obj, "init_obj is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  publisher, "publisher is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc;

  // clean-up publisher
  rc = rcl_publisher_fini(publisher, node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_publisher_fini, rcl_publisher_fini);
  }

  // de-allocate publisher itself
  init_obj->allocator->deallocate(publisher, init_obj->allocator->state);
  return rc;
}


rcl_subscription_t *
rcl_create_subscription_wrapper(
  rcl_node_t * node,
  rcl_allocator_t * allocator,
  const rosidl_message_type_support_t * type_support,
  const char * topic_name)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  node, "node is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  allocator, "allocator is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  type_support, "type_support is a null pointer", return NULL);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  topic_name, "topic_name is a null pointer", return NULL);

  rcl_subscription_t * sub = allocator->allocate( sizeof(rcl_subscription_t), allocator->state);
  (*sub) = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
  rcl_ret_t rc = rcl_subscription_init(
    sub,
    node,
    type_support,
    topic_name,
    &sub_ops);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_create_subscription_wrapper, rcl_subscription_init);
    allocator->deallocate( sub, allocator->state);
    return NULL;
  }
  return sub;
}

rcl_ret_t
rcl_subscription_fini_wrapper(rcl_init_wrapper_t * init_obj, rcl_subscription_t * subscription, rcl_node_t * node)
{
  RCL_CHECK_FOR_NULL_WITH_MSG(
  init_obj, "init_obj is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  subscription, "subscription is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  node, "node is a null pointer", return RCL_RET_INVALID_ARGUMENT);

  // de-allocate memory inside subscription
  rcl_ret_t rc = rcl_subscription_fini(subscription, node);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_subscription_wrapper, rcl_subscription_fini);
  }

  // de-allocate subscription itself
  init_obj->allocator->deallocate(subscription, init_obj->allocator->state);
  return rc;
}

rcl_timer_t *
rcl_create_timer_wrapper(
  rcl_init_wrapper_t * init_obj,
  const uint64_t timeout_ns,
  const rcl_timer_callback_t callback) {

  RCL_CHECK_FOR_NULL_WITH_MSG(
  init_obj, "node is a null pointer", return NULL);  

  rcl_ret_t rc;
  rcl_timer_t * timer = init_obj->allocator->allocate( sizeof(rcl_timer_t),
    init_obj->allocator->state);

  rc = rcl_clock_init(RCL_STEADY_TIME, &init_obj->clock, init_obj->allocator);
  if (rc != RCL_RET_OK) {
      PRINT_RCL_ERROR(rcl_create_timer_wrapper, rcl_clock_init);
      init_obj->allocator->deallocate( timer, init_obj->allocator->state );
      return NULL;
  }

  (*timer) = rcl_get_zero_initialized_timer();
  rc = rcl_timer_init(timer, &init_obj->clock, &init_obj->context, timeout_ns,
      callback, (*init_obj->allocator));
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rclc_create_timer, rcl_timer_init);
    return NULL;
  } else {
    RCUTILS_LOG_INFO("Created a timer with period %ld ms.\n", timeout_ns/1000000);
  }

  return timer;
}

/* deallocates memory of timer and the timer itself */
rcl_ret_t
rcl_timer_fini_wrapper(rcl_init_wrapper_t * init_obj, rcl_timer_t * timer) {
  RCL_CHECK_FOR_NULL_WITH_MSG(
  init_obj, "init_obj is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_FOR_NULL_WITH_MSG(
  timer, "timer is a null pointer", return RCL_RET_INVALID_ARGUMENT);
  rcl_ret_t rc;

  //de-allocate the memory within rcl_timer_t
  rc = rcl_timer_fini(timer);
  if (rc != RCL_RET_OK) {
    PRINT_RCL_ERROR(rcl_timer_fini_wrapper, rcl_timer_fini);
  }
  //de-allocate the timer itself
  init_obj->allocator->deallocate( timer, init_obj->allocator->state);
  return rc;
}

/***************************** CALLBACKS *******************************************/


void __attribute__((instrument_function))
my_callback(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (msg == NULL) {
        printf("Callback: msg NULL\n");
    } else {
        //printf("Callback: I heard: %s\n", msg->data.data);
    }
    // usleep(500000);   // sleep for 500ms
}

#define UNUSED(x) (void)x;

void __attribute__((instrument_function))
my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    rcl_ret_t rc;
    UNUSED(last_call_time);
    if (timer != NULL) {
        //printf("Timer: time since last call %d\n", (int) last_call_time);
    }

    /*
    // publish all messages
    for(unsigned int i = 0; i < NUM_PUBLISHER; i++) {
        rc = rcl_publish(pubs[i], &pub_msg1, NULL);
        if (rc != RCL_RET_OK) {
            printf("Error publishing message pubs[%d]\n", i);
        }
    }
    */
}

void timer_0_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    // todo publish the timers corresponding to the timer
}

/* helper function to parse program line */
typedef struct {
    unsigned int pubs;
    unsigned int subs;
    unsigned int * pub_names;
    unsigned int * sub_names;
} conf_node_t; 

typedef struct {
    unsigned int rate;
    unsigned int msg_size;
    unsigned int num_nodes;
    conf_node_t  * * nodes;
} conf_t;

/* parses arguments and saves configuration in conf object
   memory is allocated with calloc for a node, list of publishers, list of subscribers
   return value: 0 success -1 failure
*/
int parse_args(int argc, const char * argv[], conf_t * conf) {
 int i = 0;
    if (argc < 4) { 
        printf("Usage: %s rate message_size number_nodes [node number_publisher number_subscriber [topic_id]*]*\n", argv[0]); 
        return -1;
    }
    if (sscanf (argv[1], "%u", &conf->rate) != 1) {
        fprintf(stderr, "error - not an integer");
        return -1;
    }   

    if (sscanf (argv[2], "%u", &conf->msg_size) != 1) {
        fprintf(stderr, "error - not an integer");
        return -1;
    } 
    
    if (sscanf (argv[3], "%u", &conf->num_nodes) != 1) {
        fprintf(stderr, "error - not an integer");
        return -1;
    } 

    conf->nodes = calloc(conf->num_nodes, sizeof(conf_node_t *));

    printf("Arguments: rate %d msg_size %d #nodes %d\n", conf->rate, conf->msg_size, conf->num_nodes);
    // argv[4]='node'
    i = 4;
    for(unsigned int node_index = 0; node_index < conf->num_nodes; node_index++)
    {
        if ( strcmp( argv[i], "node") == 0) {
            conf_node_t * node = calloc(1, sizeof(conf_node_t));
            i++;

            if (sscanf (argv[i], "%u", &node->pubs) != 1) {
                fprintf(stderr, "error - #pubs not an integer");
                return -1;
            } 
            i++;

            if (sscanf (argv[i], "%u", &node->subs) != 1) {
                fprintf(stderr, "error - #subs not an integer");
                return -1;
            } 
            i++;

            if (node->pubs > 0) {
                node->pub_names = calloc(node->pubs, sizeof(unsigned int));
            }
            if (node->subs > 0) {
                node->sub_names = calloc(node->subs, sizeof(unsigned int));
            }

            for(unsigned int p=0; p<node->pubs; p++) {
                if (sscanf (argv[i], "%u", &node->pub_names[p]) != 1) {
                    fprintf(stderr, "error - topic_name (publisher) is not an integer");
                    return -1;
                } 
                i++;
            }
            for(unsigned int s=0; s<node->subs; s++) {
                if (sscanf (argv[i], "%u", &node->sub_names[s]) != 1) {
                    fprintf(stderr, "error - topic_name (subscriber) is not an integer");
                    return -1;
                } 
                i++;
            }
  
            conf->nodes[node_index] = node;
  
        } else {
            printf("Error: wrong arguments in node-configuration\n");
            return -1;
        }
    }
  return 0;
}

void print_configuration(conf_t * conf) {
    for(unsigned int node_index = 0; node_index < conf->num_nodes; node_index++)
    {
        printf("node %u ", node_index);
        
        printf("pub: ");
        for(unsigned int i=0; i < conf->nodes[node_index]->pubs; i++) {
            printf("%u ", conf->nodes[node_index]->pub_names[i]);
        }

        printf(" sub: ");
        for(unsigned int i=0; i < conf->nodes[node_index]->subs; i++) {
            printf("%u ", conf->nodes[node_index]->sub_names[i]);
        }
        printf("\n");
    }
}

/* clean-up memory of conf object
*/
int conf_fini(conf_t * conf) {

  for(unsigned int n=0; n < conf->num_nodes; n++) {
      // free list of publishers
      free (conf->nodes[n]->pub_names);
      // free list of subscribers
      free (conf->nodes[n]->sub_names);
      // free node itself
      free (conf->nodes[n]);
  }
  // free node-list
  free(conf->nodes);
}
/******************** MAIN PROGRAM **************************************/
int main(int argc, const char * argv[])
{
    rcl_ret_t rc;

    // parse command arguments
    conf_t conf;
  
    if( parse_args( argc, argv, &conf) != 0) {
       printf("Error while parsing arguments.\n");
       return -1;
    }

    print_configuration(&conf);

    conf_fini( &conf);

/*
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_wrapper_t init_obj;
    rc = rcl_init_wrapper(&init_obj, argc, argv, &allocator);
    if (rc != RCL_RET_OK) {
        printf("Error from rcl_init_wrapper. Abort\n");
        return -1;
    }


    const unsigned int TOPIC_NAME_SIZE = 18; // implementation dependent, do not change!
    char topic_name[TOPIC_NAME_SIZE];

    const unsigned int NODE_NAME_SIZE = 18; // implementation dependent, do not change!
    char node_name[NODE_NAME_SIZE];

    // create nodes
    for (unsigned int i = 0; i < NUM_NODES; i++) {
        snprintf(node_name, NODE_NAME_SIZE, "node_%d", i);
        printf("%s\n", node_name );
      nodes[i] = rcl_create_node_wrapper(node_name, "", &init_obj); 
      if ( nodes[i] == NULL) {
          printf("Error from rcl_create_node_wrapper. Abort\n");
          return -1;
      }
    }

    // xxx continue here

    // create publishers
    // assumption NUM_NODE = 10, NUM_PUBLISHER = 20 (or in general twice as many publishers as nodes)
    // every node publishes two topics
    unsigned int n; 
    unsigned int p;
    for (n = 0, p=0; n < NUM_NODES && p < NUM_PUBLISHER;p++, n++ ) {

        snprintf(topic_name, TOPIC_NAME_SIZE, "topic_%d", p);
        printf("Debug: publisher %s in node %d\n", topic_name, n );

        pubs[p] = rcl_create_publisher_wrapper(nodes[n], init_obj.allocator,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name);
        if (pubs[p] == NULL) {
            printf("Error: Could not create publisher %s\n", topic_name);
            return -1;
        } 

        // one node for two subscriptions
        p++;

        snprintf(topic_name, TOPIC_NAME_SIZE, "topic_%d", p);
        printf("Debug: publisher %s in node %d\n", topic_name, n );

        pubs[p] = rcl_create_publisher_wrapper(nodes[n], init_obj.allocator,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name);
        if (pubs[p] == NULL) {
            printf("Error: Could not create publisher %s\n", topic_name);
            return -1;
        }
    }

    // only one message string
    std_msgs__msg__String__init(&pub_msg1);
    rosidl_generator_c__String__assignn(&pub_msg1, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 68);

    // subscriber
    unsigned int s;
    unsigned int p;
    unsigned int n=0;
    for (s=0, p=0; s < NUM_SUBSCRIBER; s++) {

        snprintf(topic_name, TOPIC_NAME_SIZE, "topic_%d", p);

        subs[s] = rcl_create_subscription_wrapper(nodes[n], init_obj.allocator,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
            topic_name);

        if (subs[s] == NULL) {
            printf("Error: Could not create subscriber %s\n", topic_name);
        } else {
            printf("sub[%d] with %s\n", s, topic_name);
        }

        std_msgs__msg__String * msg = init_obj.allocator->allocate( 
            sizeof(std_msgs__msg__String), init_obj.allocator->state); 
        std_msgs__msg__String__init( msg );
        subs_msg[s] = msg;
    }

    // todo: for each node create one timer
    const unsigned int timer_timeout = 20; // in milli-seconds
    printf("ROS created a node with %d publishers @ %d ms\n", NUM_PUBLISHER, timer_timeout);
    rcl_timer_t * timer1 = rcl_create_timer_wrapper(&init_obj,
      RCL_MS_TO_NS(timer_timeout), my_timer_callback); 

    rcle_let_executor_t exe;
    rcle_let_executor_init(&exe, &init_obj.context, (NUM_SUBSCRIBER + NUM_TIMER), 
      init_obj.allocator);

    for(unsigned int i = 0; i < NUM_SUBSCRIBER; i++){
        rc = rcle_let_executor_add_subscription(&exe, subs[i], subs_msg[i], &my_callback,
            ON_NEW_DATA);
        if (rc != RCL_RET_OK) { 
            printf("Error: Could not add subscription %d to executor.\n", i);
        }
    }

    rcle_let_executor_add_timer(&exe, timer1);
    if (rc != RCL_RET_OK) { 
        printf("Error rcle_let_executor_add_timer: Could not add timer to executor");
    }

    // set time_out for rcl_wait() in nanoseconds 100ms = 100 000 000 ns
    unsigned int rcl_wait_timeout = 1000; 
    rc = rcle_let_executor_set_timeout(&exe,  RCL_MS_TO_NS(rcl_wait_timeout));
    if (rc != RCL_RET_OK) {
        printf("Error rcle_let_executor_set_timeout: Could not configure timeout to executor");
    }

    // spin forever
    rcle_let_executor_spin(&exe);

    // clean up
    rc = rcle_let_executor_fini(&exe);

    for (unsigned int i = 0 ; i < NUM_PUBLISHER; i++) {
        rc = rcl_publisher_fini_wrapper(&init_obj, pubs[i], node0);
    }
    
    for (unsigned int i = 0 ; i < NUM_SUBSCRIBER; i++) {
        rc = rcl_subscription_fini_wrapper(&init_obj, subs[i], node0);
    }

    for (unsigned int i = 0 ; i < NUM_SUBSCRIBER; i++) {
        init_obj.allocator->deallocate( subs_msg[i], init_obj.allocator->state);
    }

    rc = rcl_timer_fini_wrapper(&init_obj, timer1);
    rc = rcl_node_fini_wrapper(&init_obj, node0);
    rc = rcl_init_options_fini(&init_obj);
*/
    return 0;
    
}
