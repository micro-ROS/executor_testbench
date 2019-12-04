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
#include "rcl_wrapper/rcl_wrapper.h"
#include <std_msgs/msg/string.h>
// #include <unistd.h> // for usleep

/* TODO
+ one timer for each publisher
- finish after number of published /subscribed topics
- documentation
*/



/* helper data structure for rcl interface 
   to save the node with its publishers and subscribers
*/
typedef struct {
  rcl_node_t * rcl_node;  // rcl_node
  unsigned int id; // node id
  unsigned int num_pubs; // number of publishers
  unsigned int num_subs; // number of subscribers
  rcl_publisher_t ** pubs;  // list of publishers (length = num_pubs)
  rcl_timer_t ** timers;  // list of timer pointers  (length = num_pubs)
  unsigned int * pub_names; // list of topic ids of the publishers (length = num_pubs)
  rcl_subscription_t ** subs;  // list of subscriber pointers (length = num_subs)
  std_msgs__msg__String ** subs_msg; // list of msgs of the subscribers
  unsigned int * published_msgs; // list of counters for published messages
} rcl_node_wrapper_t;


/* helper function to parse program line */
typedef struct {
    unsigned int id;
    unsigned int num_pubs;
    unsigned int num_subs;
    unsigned int * pub_names;
    unsigned int * sub_names;
} conf_node_t; 

typedef struct {
    unsigned int rate;  // Hz
    // testbench finishes when all topics have been published 'max_published_msg' times
    // and all subscribed topics have been received 'max_subscribed_msg' times.
    // these values can be zero.
    unsigned int max_published_msg; // number of messages per topic to publish
    unsigned int max_subscribed_msg; // number of messages per topic to receive
    unsigned int period;  // rate converted into ms
    unsigned int msg_size;  // number of char
    unsigned int num_nodes;
    conf_node_t  * * nodes;
    unsigned int total_num_pubs;
    unsigned int total_num_subs;
} conf_t;

static 
void initialize_config(conf_t * conf){
  conf->rate = 0;
  conf->max_published_msg = 0;
  conf->max_subscribed_msg = 0;
  conf->period = 0;
  conf->msg_size = 0;
  conf->num_nodes = 0;
  conf->nodes = NULL;
  conf->total_num_pubs = 0;
  conf->total_num_subs = 0;
}
/* parses arguments and saves configuration in conf object
   memory is allocated with calloc for a node, list of publishers, list of subscribers
   call configuration_fini() to free memory again.

   return value: 0 success -1 failure
*/
enum Testbench_arguments{PROG_NAME, RATE, MAX_PUB, MAX_SUB, MSG_SIZE, NUM_NODES, NODE_CONFIGS};
int parse_args(int argc, const char * argv[], conf_t * conf) {
  int i = 0;
  if (argc < 4) { 
    printf("Error: too few arguments missing: %d\n", 4-argc);
    printf("Usage: rate max_published_msg max_subscribed_msg message_size number_nodes \
    ['node' id number_publisher number_subscriptions [topic_id]*]*\n \n \
    Note: for each node specify node-id, number of publishers(p) and number subscriptions(s). \n \
    Then follows a ordered list of topic-ids. The first p topic-id are topic names for the \
    publishers, the next s topic-ids are topic name for the subscriptions.\n \
    Example 1: node 0 1 1 3 4 => node_0 with one publisher and one subscriptions \
    publisher topic='topic_3', subscription topic='topic_4'\nExample 2: node 1 2 0 3 4 => \
    node_1 with two publishers with the topic names 'topic_3' and 'topic_4'.\n"); 
    return -1;
  }

  initialize_config(conf);
  if (sscanf (argv[RATE], "%u", &conf->rate) != 1) {
    fprintf(stderr, "error - rate is not an unsigned integer \n");
    return -1;
  }   

  if ( conf->rate == 0) { 
    printf("Error: rate must be positive.");
    return -1;
  }
  conf->period = (1000)/ conf->rate; // in milliseconds 
  // conversion errors => TODO use nanoseconds, but 1000 * 1000 * 1000 is too large for unsigned int!

  if (sscanf (argv[MAX_PUB], "%u", &conf->max_published_msg) != 1) {
    fprintf(stderr, "error - max_published_msg is not an integer.\n");
    return -1;
  }  

  if (sscanf (argv[MAX_SUB], "%u", &conf->max_subscribed_msg) != 1) {
  fprintf(stderr, "error - max_subscribed_msg is not an integer.\n");
  return -1;
}  

  if (sscanf (argv[MSG_SIZE], "%u", &conf->msg_size) != 1) {
    fprintf(stderr, "error - msg_size not an integer");
    return -1;
  } 
  if (sscanf (argv[NUM_NODES], "%u", &conf->num_nodes) != 1) {
    fprintf(stderr, "error - num_nodes not an integer");
    return -1;
  } 

  conf->nodes = calloc(conf->num_nodes, sizeof(conf_node_t *));
 
  i = NODE_CONFIGS;
  for(unsigned int node_index = 0; node_index < conf->num_nodes; node_index++) {
    if ( strcmp( argv[i], "node") == 0) {
      conf_node_t * node = calloc(1, sizeof(conf_node_t));
      i++;

      if (sscanf (argv[i], "%u", &node->id) != 1) {
        fprintf(stderr, "error - node-id not an integer");
        return -1;
      } 
      i++;

      if (sscanf (argv[i], "%u", &node->num_pubs) != 1) {
        fprintf(stderr, "error - #pubs not an integer");
        return -1;
      }
      conf->total_num_pubs += node->num_pubs;
      i++;

      if (sscanf (argv[i], "%u", &node->num_subs) != 1) {
        fprintf(stderr, "error - #subs not an integer");
        return -1;
      } 
      conf->total_num_subs += node->num_subs;
      i++;

      if (node->num_pubs > 0) {
        node->pub_names = calloc(node->num_pubs, sizeof(unsigned int));
      }
      if (node->num_subs > 0) {
        node->sub_names = calloc(node->num_subs, sizeof(unsigned int));
      }

      for(unsigned int p=0; p<node->num_pubs; p++) {
        if (sscanf (argv[i], "%u", &node->pub_names[p]) != 1) {
          fprintf(stderr, "Error: topic_name (publisher) is not an integer");
          return -1;
        } 
        i++;
      }
      for(unsigned int s=0; s<node->num_subs; s++) {
        if (sscanf (argv[i], "%u", &node->sub_names[s]) != 1) {
          fprintf(stderr, "Error: topic_name (subscriber) is not an integer");
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

static
void print_configuration(conf_t * conf) {
  printf("Arguments: rate %d max_pub %d max_sub %d msg_size %d #nodes %d\n", conf->rate, 
     conf->max_published_msg, conf->max_subscribed_msg, conf->msg_size, conf->num_nodes);
  printf("Total number pubs: %d\n", conf->total_num_pubs);
  printf("Total number subs: %d\n", conf->total_num_subs);

  for(unsigned int node_index = 0; node_index < conf->num_nodes; node_index++)
  {
    printf("node %u ", conf->nodes[node_index]->id);
    
    printf("pub: ");
    for(unsigned int i=0; i < conf->nodes[node_index]->num_pubs; i++) {
      printf("%u ", conf->nodes[node_index]->pub_names[i]);
    }

    printf(" sub: ");
    for(unsigned int i=0; i < conf->nodes[node_index]->num_subs; i++) {
      printf("%u ", conf->nodes[node_index]->sub_names[i]);
    }
    printf("\n");
  }
}

/* clean-up memory of conf object
*/
static 
void configuration_fini(conf_t * conf) {
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

/* testbench for rcl_executor
   - generation of rcl_nodes with publishers and subscribers
   - one timer is created for each publisher
   - rcl_executor is configured with all subscribers of all nodes
   - rcl_executor.spin runs endlessly

   parameters: 
   - publsish rate (for all publishers)
   - message size of publsihed message
   - number of nodes with specific topic_names of publishers and subscribers
*/  

// global data structures
static rcl_node_wrapper_t * nodes;  // list of nodes
static unsigned int num_nodes;  // necessary in timer_callback
std_msgs__msg__String pub_msg;  // one message string for all publishers with user-defined length

typedef struct {
// for testbench management (i.e when to stop testbench)
  unsigned int max_published_msgs; // configured
  unsigned int number_publishers; // configured
  unsigned int max_received_msgs; // configured
  unsigned int finished_publishers; // status var
  unsigned int received_msgs; // status var
} test_setup_t;

static test_setup_t test_setup;
/***************************** CALLBACKS ***********************************/


void __attribute__((instrument_function))
subscriber_callback(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (msg == NULL) {
        printf("Callback: msg NULL\n");
    } else {
        printf("Callback: I heard: %s\n", msg->data.data);
        test_setup.received_msgs++;
    }
    // usleep(500000);   // sleep for 500ms
}

#define UNUSED(x) (void)x;

void __attribute__((instrument_function))
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  UNUSED(last_call_time);
  if (timer != NULL) {
    //  printf("Timer: time since last call %d\n", (int) last_call_time);

    // find corresponding timer and publish its messages
    // in each node n the list of publishers is mirrored by the list of timers:
    // the timer nodes[n].timers[i] corresponds to the publisher nodes[n].pubs[i]
    bool found = false;
    for(unsigned int n = 0; n<num_nodes && found == false; n++) {
      //  printf("Timer callback: searching timer in node %u\n", n);
      for(unsigned int i=0; i<nodes[n].num_pubs && found == false; i++) {
        if (nodes[n].timers[i] == timer) {
          found = true;
          if (nodes[n].published_msgs[i] < test_setup.max_published_msgs)              
          {
            rc = rcl_publish(nodes[n].pubs[i], &pub_msg, NULL);
            nodes[n].published_msgs[i]++;

            // when all messages have been sent, then increment number of finished publisher
            // this is only incremented once per publisher, as in the next call to this 
            // timer_callback the above if-clause will evaluate false. 
            if (nodes[n].published_msgs[i] == test_setup.max_published_msgs) {
              test_setup.finished_publishers++;
            }

            //  debug output
            if (rc != RCL_RET_OK) {
                printf("Error publishing message node[%u].pub[%u]\n", n, i);
            } else {
                printf("node %u: published topic_%u\n", nodes[n].id, nodes[n].pub_names[i]);
            }
          }
        }
      }
    }
  } else {
    printf("Error: timer_callback is NULL\n");
  }
}


static void initialize_testbench(conf_t * conf) {
  // define how many messages to send for each publisher
  test_setup.max_published_msgs = conf->max_published_msg;
  // total number of publishers
  test_setup.number_publishers = conf->total_num_pubs;
  // define how many message to receive for all subscribers together
  test_setup.max_received_msgs = conf->total_num_subs * conf->max_subscribed_msg;
  // initialize status variables
  test_setup.finished_publishers = 0;
  test_setup.received_msgs = 0;

  printf("test_setup.max_published_msgs = %d\n",  test_setup.max_published_msgs);
  printf("test_setup.number_publishers = %d\n", test_setup.number_publishers);
  printf("test_setup.max_received_msgs = %d\n", test_setup.max_received_msgs);
}
/* returns true, if
  - each publisher has sent 'conf->max_published_msg' messages AND
  - in total 'conf->total_num_subs * conf->max_subscribed_msg' have been received

  otherwise false. 
  
  Currently it is not possible to count the number of received msg for each subscriber, because there is only one 
  subscription callback for all subscribers. The only parameter is the message itself. As a quick fix, I am counting
  the total number of received messages.
  
  The expected finish-criteria, that each subscriber has received conf->max_subscribed_msg, is not implemented, but only 
  the relaxed criteria that all subscribers together have received conf->max_subscribed_msg * conf->total_num_pubs
  
*/

static bool work_to_do() {
  if (test_setup.finished_publishers < test_setup.number_publishers ||
      test_setup.received_msgs <  test_setup.max_received_msgs)
    return true;
  else 
    return false;
}

/******************** MAIN PROGRAM *****************************************/
int main(int argc, const char * argv[]) {
  rcl_ret_t rc;
  conf_t conf;

  ////////////////////////////////////////////////////////////////////////////
  // Parse configuration 
  ////////////////////////////////////////////////////////////////////////////
  if( parse_args( argc, argv, &conf) != 0) {
    printf("Error while parsing arguments.\n");
    return -1;
  }

  // just for debugging
  print_configuration(&conf);

  ////////////////////////////////////////////////////////////////////////////
  // Initialize RCL
  ////////////////////////////////////////////////////////////////////////////
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_wrapper_t init_obj;
  rc = rcl_init_wrapper(&init_obj, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
      printf("Error in rcl_init_wrapper.\n");
      return -1;
  }

  ////////////////////////////////////////////////////////////////////////////
  // Configure rcl handles (nodes, publishers, timers, subscriptions)
  ////////////////////////////////////////////////////////////////////////////

  // create fixed strings for node_name and topic_name 
  const unsigned int NODE_NAME_SIZE = 18; // implementation dependent, do not change!
  char node_name[NODE_NAME_SIZE];
  const unsigned int TOPIC_NAME_SIZE = 18; // implementation dependent, do not change!
  char topic_name[TOPIC_NAME_SIZE];

  // GLOBAL DATA STRUCTURES
  //create list of nodes
  nodes = calloc ( conf.num_nodes, sizeof(rcl_node_wrapper_t));
  num_nodes = conf.num_nodes;

  //create nodes
  for (unsigned int n = 0; n < conf.num_nodes; n++) {
    snprintf(node_name, NODE_NAME_SIZE, "node_%u", conf.nodes[n]->id);
    printf("Debug: creating %s\n", node_name );
    nodes[n].rcl_node = rcl_create_node_wrapper(node_name, "", &init_obj); 
    if ( nodes[n].rcl_node == NULL) {
        printf("Error in rcl_create_node_wrapper.\n");
        return -1;
    }
    nodes[n].id = conf.nodes[n]->id;

    // add publishers
    if ( conf.nodes[n]->num_pubs > 0) {
      nodes[n].num_pubs = conf.nodes[n]->num_pubs;
      nodes[n].pubs =  calloc ( conf.nodes[n]->num_pubs, sizeof(rcl_publisher_t*));
      nodes[n].pub_names =  calloc ( conf.nodes[n]->num_pubs, sizeof(unsigned int));
      nodes[n].timers =  calloc ( conf.nodes[n]->num_pubs, sizeof(rcl_timer_t*));
      nodes[n].published_msgs =  calloc ( conf.nodes[n]->num_pubs, sizeof(int));

      for(unsigned int p = 0; p < conf.nodes[n]->num_pubs; p++) {
        snprintf(topic_name, TOPIC_NAME_SIZE, "topic_%u", conf.nodes[n]->pub_names[p]);
        printf("  ... publishes %s\n", topic_name);
        nodes[n].pub_names[p] = conf.nodes[n]->pub_names[p];
        nodes[n].pubs[p] = rcl_create_publisher_wrapper(nodes[n].rcl_node, 
          init_obj.allocator, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
          topic_name);
        if (nodes[n].pubs[p] == NULL) {
          printf("Error: Could not create publisher %s.\n", topic_name);
          return -1;
        }
        nodes[n].timers[p] = rcl_create_timer_wrapper(&init_obj,
          RCL_MS_TO_NS(conf.period), timer_callback); 
      }
    }

    // add subscriptions
    if ( conf.nodes[n]->num_subs > 0) {
      nodes[n].num_subs = conf.nodes[n]->num_subs;  // needed???
      nodes[n].subs =  calloc ( conf.nodes[n]->num_subs, sizeof(rcl_subscription_t*));
      nodes[n].subs_msg =  calloc ( conf.nodes[n]->num_subs, sizeof(std_msgs__msg__String*));

      for(unsigned int s = 0; s < conf.nodes[n]->num_subs; s++) {
        snprintf(topic_name, TOPIC_NAME_SIZE, "topic_%u", conf.nodes[n]->sub_names[s] );
        printf(" ... subscribes %s\n", topic_name );
        nodes[n].subs[s] = rcl_create_subscription_wrapper(nodes[n].rcl_node, init_obj.allocator,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), topic_name);
        if (nodes[n].subs[s] == NULL) {
          printf("Error: Could not create subscriber %s.\n", topic_name);
          return -1;
        } 

        std_msgs__msg__String * msg = init_obj.allocator->allocate( 
          sizeof(std_msgs__msg__String), init_obj.allocator->state); 
        std_msgs__msg__String__init( msg );
        nodes[n].subs_msg[s] = msg;
      }
    }


  }

  // default message string for all publishers
  std_msgs__msg__String__init(&pub_msg);
  char * pub_string = malloc(conf.msg_size);
  for(unsigned int i=0; i<conf.msg_size; i++) {
    pub_string[i] = 'a';
  }
  rosidl_generator_c__String__assignn(&pub_msg, pub_string, conf.msg_size);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  rcle_let_executor_t exe;

  //compute total number of subsribers and timers
  unsigned int num_handles = 0; 
  for(unsigned int i=0; i< conf.num_nodes;i++){
    num_handles += conf.nodes[i]->num_subs;
    num_handles += conf.nodes[i]->num_pubs;
  }

  printf("Debug: number of handles: %u\n", num_handles);
  rcle_let_executor_init(&exe, &init_obj.context, num_handles, init_obj.allocator);
  
  // set timeout for rcl_wait() 
  const unsigned int RCL_WAIT_TIMEOUT = 1000; //ms => 1s
  rc = rcle_let_executor_set_timeout(&exe, RCL_MS_TO_NS(RCL_WAIT_TIMEOUT));
  if (rc != RCL_RET_OK) {
      printf("Error in rcle_let_executor_set_timeout.");
  }

  for(unsigned int n = 0; n<conf.num_nodes; n++){
    // add one timer per publisher
    for(unsigned int p=0; p<nodes[n].num_pubs;p++) {
      rc = rcle_let_executor_add_timer(&exe, nodes[n].timers[p]);
      if (rc != RCL_RET_OK) { 
          printf("Error rcle_let_executor_add_timer: Could not add timer to executor: nodes[%u].timers[%u]\n",n, p);
          PRINT_RCL_ERROR(main, rcle_let_executor_add_timer);
      }
    }

    // add subscription
    for(unsigned int s=0; s<nodes[n].num_subs;s++) {
      rc = rcle_let_executor_add_subscription(&exe, nodes[n].subs[s], nodes[n].subs_msg[s], 
        &subscriber_callback, ON_NEW_DATA);
      if (rc != RCL_RET_OK) { 
        printf("Error in rcle_let_executor_add_subscription node[%u] sub %u \n", n, s);
        PRINT_RCL_ERROR(main, rcle_let_executor_add_subscription);
      }
    }
  }
  ////////////////////////////////////////////////////////////////////////////
  // Run RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  initialize_testbench( &conf );

  while (work_to_do()) {
    rcle_let_executor_spin_some(&exe, RCL_WAIT_TIMEOUT);
  }
  ////////////////////////////////////////////////////////////////////////////
  // Clean up - free dynamically allocated memory
  ////////////////////////////////////////////////////////////////////////////
  
  // executor
  rc = rcle_let_executor_fini(&exe);
  
  // rcl handles
  for (unsigned int n = 0; n < conf.num_nodes; n++) {
    // delete memory of publishers and timers
    if ( conf.nodes[n]->num_pubs > 0) {
      for(unsigned int i = 0; i < conf.nodes[n]->num_pubs; i++) {
        rc = rcl_publisher_fini_wrapper(&init_obj, nodes[n].pubs[i], nodes[n].rcl_node);
        rc = rcl_timer_fini_wrapper(&init_obj, nodes[n].timers[i]);
      }
      free (nodes[n].pubs);
      free (nodes[n].pub_names);
      free (nodes[n].timers);
      free (nodes[n].published_msgs);
    }

    // delete memory of subscriptions
    if ( conf.nodes[n]->num_subs > 0) {
      for(unsigned int i = 0; i < conf.nodes[n]->num_subs; i++) {
        rc = rcl_subscription_fini_wrapper(&init_obj, nodes[n].subs[i], nodes[n].rcl_node);
        init_obj.allocator->deallocate(nodes[n].subs_msg[i], init_obj.allocator->state);
      }
      free(nodes[n].subs);
      free(nodes[n].subs_msg);
    }
    // delete memory of node
    rc = rcl_node_fini_wrapper(&init_obj, nodes[n].rcl_node);
  }
  free (pub_string);
  rc = rcl_init_fini_wrapper(&init_obj);
  free (nodes);
  // configuration object (command line arguments)
  configuration_fini( &conf);
  
  return 0;
  }