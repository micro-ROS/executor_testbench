'''Configure test cases for ROS Executor.
Parameters:
- package
- binary
- rate [Hz]
- num-published-msg [int]
- msg-size [int]
- topology-type [A,B,C,D,E,F]
- process-type [one, pub-sub, all]
- number-topics [int]
- [number-subscriber] [int] 
'''

import sys
import subprocess

# it is sufficient to specify number of published messages,
# the number of received msg will be infered.
# it depends on
# - topology type
# - process-type one, pub-sub, all
# => save them in the node - rather then as global variables

def print_node(node):
    (ID, PUB, SUB, PUB_MSG, REC_MSG) = range(5) 
    print("ID ", node[ID])
    print("PUB", node[PUB])
    print("SUB", node[SUB])

def topology(type, process_type, num_topics, num_pub_msg, m=0):
    '''Configuration of test-cases.
       parameter: enum[A-F] type, enum[all, one, pub-sub] process_type, int num_topics, int num_pub_msg, int m
       retuns:
    '''
    (ID, PUB, SUB, PUB_MSG, REC_MSG) = range(5)
    node_list = []
    empty_node = [0,[],[],0,0]
    pub_list = []
    sub_list = []   
    node_id = 0 

    if num_topics < 0:
        raise ValueError('num_topics must be non-negative')

    if type == 'A':
        print("Type: A")
        node =  empty_node.copy()
        node[ID] = node_id
        node_id += 1
        pub_list = []
        if (num_topics != 1):
            raise ValueError('num_topics must be equal to one for topology type A')
        for i in range(num_topics):
            pub_list.append(str(i))
        sub_list = []
        node[PUB]=pub_list
        node[SUB]=sub_list
        node[PUB_MSG]=int(num_pub_msg)
        node[REC_MSG]=0
        node_list.append(node)

        for i in range(m):
            node =   empty_node.copy()
            node[ID] = node_id
            node_id +=1
            pub_list = []
            for s in range (num_topics):
                sub_list = [str(s)]
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=0
            node[REC_MSG]=int(num_pub_msg)
            node_list.append(node)


    elif type =='B':
        print("Type: B")
        node = empty_node.copy()
        node[ID] = node_id
        node_id +=1
        pub_list = []
        for i in range(num_topics):
            pub_list.append(str(i))
        sub_list = []
        node[PUB]=pub_list
        node[SUB]=sub_list
        node[PUB_MSG]=int(num_pub_msg)
        node[REC_MSG]=0
        node_list.append(node)

        for i in range(num_topics):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1
            pub_list = []
            sub_list = [str(i)]
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=0
            node[REC_MSG]=int(num_pub_msg)
            node_list.append(node)


    elif type == 'C':
        print("Type: C")
        if (num_topics != 1):
            raise ValueError('num_topics must be equal to one for topology type C')
        # add publisher nodes
        num_publishers = m   
        num_subscribers = 1 
        for p in range(num_publishers):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1
            pub_list = []
            sub_list = []
            for i in range (num_topics):
                pub_list.append(str(i))
            
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=int(num_pub_msg)
            node[REC_MSG]=0
            node_list.append(node)
        
        # add subscriber nodes
        for s in range(num_subscribers):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1
            pub_list = []
            sub_list = []
            for i in range(num_topics):
                sub_list.append(str(i))

            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=0
            node[REC_MSG]=int(num_pub_msg) * int(num_publishers)
            node_list.append(node)


    elif type == 'D':
        print("Type: D")
        # add publisher nodes
        num_publishers=num_topics
        for p in range(num_publishers):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=int(num_pub_msg)
            node[REC_MSG]=int(0)
            node_list.append(node)
        
        # add subscriber node
        node = empty_node.copy()
        node[ID] = node_id
        node_id +=1        
        pub_list = []
        sub_list = []
        for s in range(num_topics):
            sub_list.append(str(s))

        node[PUB]=pub_list
        node[SUB]=sub_list
        node[PUB_MSG]=0
        node[REC_MSG]=int(num_pub_msg) * int(num_publishers)
        node_list.append(node)
    
    elif type == 'E':
        print("Type: E")
        # publishers
        num_publishers=num_topics
        num_subscribers=m
        for p in range(num_publishers):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=int(num_pub_msg)
            node[REC_MSG]=int(0)
            node_list.append(node)
        # subscribers
        for s in range(num_subscribers):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            for t in range (num_publishers):
                sub_list.append(str(t))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node[PUB_MSG]=int(0)
            node[REC_MSG]=int(num_publishers)*int(num_pub_msg)
            node_list.append(node)
    
    elif type == 'F':    
        print("Type: F")
        # one node with 
        #     #publishers = #topics
        #     m = #subscribers per topics 

        node = empty_node.copy()
        node[ID] = node_id
        node_id +=1        
        pub_list = []
        sub_list = []

        num_publishers=num_topics
        num_subscribers=m
        # add publishers
        for p in range(num_publishers):
            pub_list.append(str(p))
        node[PUB]=pub_list
        
        # add subscribers

        for s in range(num_subscribers):
            for t in range (num_publishers):
                sub_list.append(str(t))
        node[SUB]=sub_list
        node[PUB_MSG]=int(num_pub_msg)
        node[REC_MSG]=int(num_publishers)*int(num_pub_msg)*int(num_subscribers)
        node_list.append(node)  
    else:
        raise ValueError("type unknown. Possible options: [A,B,C,D,E,F]")


    # debug
    #print(node_list)

    #print("{} {}".format("process_type:", process_type))
    process_list = []
    # put them into different processes
    if process_type == 'one':
        print("Process type: all nodes are in one process.")
        # nothing needs to be done: node_list can be taken as-is
        process_list = [ node_list ]
    elif process_type == 'all':
        print("Process type: every node is executed its own process.")
        process_list=[]
        for node in node_list:
            process_list.append( [node] )
    elif process_type == 'pub-sub':
        print("Process type: one process for all publishing nodes, one process for all subscribing nodes")
        process_list=[[],[]]
        (PUB_NODES, SUB_NODES) = range(2)
        for node in node_list:
            if len(node[PUB])>0 and len(node[SUB])==0:
                process_list[PUB_NODES].append(node)
            if len(node[PUB])==0 and len(node[SUB])>0:
                 process_list[SUB_NODES].append(node)
            if len(node[PUB])>0 and len(node[SUB])>0:
                raise ValueError("Error with process_type='pub-sub' a node which publishes and subscribes at the same time, is not supported.")
    else:
        raise ValueError('process_type unknown. Possible options: [one, pub-sub, all]')

    # debugging output
    #print(process_list)

    # create API command
    # do I need str() for all the arguments?
    # add node_id in testbench_executor program
    # udpate documentation in README.md
    # - clarify pub/sub => publishing node , subscribing node
    # number timer = number publishers
    print("Generated configuration:")
    cmd_list=[]
    for nodes in process_list:
        cmd = []
        cmd.append (str(len(nodes)))
        for node in nodes:
            #print("{} {}".format("node", node))
            cmd.append("node")
            cmd.append(str(node[ID]))
            cmd.append(str(node[PUB_MSG]))
            cmd.append(str(node[REC_MSG]))
            cmd.append(str(len(node[PUB])))
            cmd.append(str(len(node[SUB])))
            print("{} {} {} {}".format("node", node[ID] , len(node[PUB]) , len(node[SUB])))
            for pub in node[PUB]:
                print("{}{}".format("  publishes ",pub))
                #publisher topic id
                cmd.append(str(pub))
            for sub in node[SUB]:
                print("{}{}".format("  subscriber ", sub))
                cmd.append(str(sub))
        cmd_list.append (cmd)

    return cmd_list


# remember to update documentation at the top of the file as well.
def main():
    if len(sys.argv[1:]) < 8:
        print("Missing arguments.")
        print("args: executor-package executor-binary rate(Hz) num-published-msg(int) msg-size(int) topology-type(A-F) \n \
        process-type(one,all,pub-sub) number-topics(int) [number-subscribers(int)]")
        return


    cmd_package = sys.argv[1] # package of ros2 binary
    cmd_name = sys.argv[2] # program name of ros2 binary
    rate =  sys.argv[3]     # in Hz
    num_pub_msg = sys.argv[4] # int
    msg_size = sys.argv[5]  # in number characters
    topology_type = sys.argv[6] # A - E
    process_type = sys.argv[7] # one, pub_sub, all - determines if a node shall run in its own process
    num_topics = int(sys.argv[8]) # int
    
    if len(sys.argv[1:])== 9:
        num_subs = int(sys.argv[9])
    else:
        num_subs = 0
    #print("{} {} {} {} {} {} {} {} {}".format(cmd_package, cmd_name, rate, num_pub_msg, msg_size, topology_type, process_type, num_topics, num_subs))

    cmd_list = topology(topology_type, process_type, num_topics, num_pub_msg, num_subs)


# continue here
# need to configure the num_pub num_rec msg per use-case because
# e.g. when a node only publishes num_rec_msg = 0
    # call executor
    for cmd_args in cmd_list:
        my_cmd = []
        my_cmd.append("ros2")
        my_cmd.append("run")
        my_cmd.append(cmd_package)
        my_cmd.append(cmd_name)
        my_cmd.append(rate)
        my_cmd.append(msg_size)
        for arg in cmd_args:
            my_cmd.append(arg)
        print(my_cmd)
        subprocess.Popen( my_cmd)
    return

if __name__ == '__main__':
    main()


        

