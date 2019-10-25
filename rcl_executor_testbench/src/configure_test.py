'''Configure test cases for ROS Executor.
Parameters:
- executor-binary [path_to/executor_testbench_binary] 
- rate [Hz] 
- msg-size [int]
- topology-type [A,B,C,D,E,F]
- process-type [one, pub-sub, all]
- number-topics [int]
- [number-subscriber] [int] 



'''
import sys
import subprocess

def topology(type, process_type, num_topics, m=0):
    '''Configure test case for ROS2 Executor.
       parameter: type: A-E num_topics: number of topics
       retuns:
    '''
    (PUB, SUB, ID) = range(3) # used as index for node and process, therefore order matters!
    node_list = []
    empty_node = [0, [] ,[] ]
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
        pub_list = []
        for i in range(num_topics):
            pub_list.append(str(i))
        sub_list = []
        node[PUB]=pub_list
        node[SUB]=sub_list
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
        node_list.append(node)

        for i in range(num_topics):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1
            pub_list = []
            sub_list = [str(i)]
            node[PUB]=pub_list
            node[SUB]=sub_list
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
            node_list.append(node)


    elif type == 'D':
        print("Type: D")
        # add publisher nodes

        for p in range(num_topics):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)
        
        # add subscriber nodes
        node = empty_node.copy()
        node[ID] = node_id
        node_id +=1        
        pub_list = []
        sub_list = []
        for s in range(num_topics):
            sub_list.append(str(s))

        node[PUB]=pub_list
        node[SUB]=sub_list
        node_list.append(node)
    
    elif type == 'E':
        print("Type: E")
        # publishers
        for p in range(num_topics):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)
        # subscribers
        print("m " + str(m))
        for s in range(m):
            node = empty_node.copy()
            node[ID] = node_id
            node_id +=1            
            pub_list = []
            sub_list = []
            for t in range (num_topics):
                sub_list.append(str(t))
            node[PUB]=pub_list
            node[SUB]=sub_list
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

        # add publishers
        for p in range(num_topics):
            pub_list.append(str(p))
        node[PUB]=pub_list
        
        # add subscribers
        # m <=> number of subscribers per topic
        print("m " + str(m))
        for s in range(m):
            for t in range (num_topics):
                sub_list.append(str(t))
        node[PUB]=pub_list
        node[SUB]=sub_list
        node_list.append(node)  
    else:
        raise ValueError("type unknown. Possible options: [A,B,C,D,E,F]")


    print("{} {}".format("process_type:", process_type))
    process_list = []
    # put them into different processes
    if process_type == 'one':
        print("All nodes are in one process.\n")
        # nothing needs to be done: node_list can be taken as-is
        process_list = [ node_list ]
    elif process_type == 'all':
        print("Every node is executed its own process.\n")
        process_list=[]
        for node in node_list:
            process_list.append( [node] )
    elif process_type == 'pub-sub':
        print("One process for all publisher nodes, one process for all subscriber nodes\n")
        process_list=[[],[]]
        for node in node_list:
            if len(node[PUB])>0 and len(node[SUB])==0:
                process_list[PUB].append(node)
            if len(node[PUB])==0 and len(node[SUB])>0:
                 process_list[SUB].append(node)
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
    cmd_list=[]
    for nodes in process_list:
        cmd = []
        cmd.append (str(len(nodes)))
        for node in nodes:
            #print("{} {}".format("node", node))
            cmd.append("node")
            cmd.append(str(node[ID]))
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



def main():
    if len(sys.argv[1:]) < 6:
        print("Missing arguments.")
        print("args: executor-binary rate(Hz) msg-size(int) topology-type(A-F) \n \
     process-type(one,all,pub-sub) number-topics(int) [number-subscribers(int)]")
        return

    cmd_name = sys.argv[1] # binary name
    rate =  sys.argv[2]     # in Hz
    msgSize = sys.argv[3]  # in number characters
    topology_type = sys.argv[4] # A - E
    process_type = sys.argv[5] # one, pub_sub, all - determines if a node shall run in its own process
    num_topics = int(sys.argv[6]) # int
    
    if len(sys.argv[1:])== 7:
        num_subs = int(sys.argv[7])
    else:
        num_subs = 0
    print("{} {} {} {} {} {} {}".format(cmd_name, rate, msgSize, topology_type, process_type, num_topics, num_subs))

    cmd_list = topology(topology_type, process_type, num_topics, num_subs)

    # call executor
    for cmd_args in cmd_list:
        my_cmd = []
        my_cmd.append(cmd_name)
        my_cmd.append(rate)
        my_cmd.append(msgSize)
        for arg in cmd_args:
            my_cmd.append(arg)
        print(my_cmd)
        subprocess.Popen( my_cmd)
if __name__ == '__main__':
    main()


        

