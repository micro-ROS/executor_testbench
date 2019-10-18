'''Configure test cases for ROS Executor.

Available functions:


'''
import sys
import subprocess

def topology(type, comm_type, num_topics, m=0):
    '''Configure test case for ROS2 Executor.
       parameter: type: A-E num_topics: number of topics
       retuns:
    '''
    (PUB, SUB) = range(2)
    node_list = []
    node = [ [] ,[] ]
    pub_list = []
    sub_list = []    

    if num_topics < 0:
        raise ValueError('num_topics must be non-negative')

    if type == 'A':
        print("Type: A")
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
            node = [[],[]]
            pub_list = []
            for s in range (num_topics):
                sub_list = [str(s)]
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)


    elif type =='B':
        print("Type: B")
        pub_list = []
        for i in range(num_topics):
            pub_list.append(str(i))
        sub_list = []
        node[PUB]=pub_list
        node[SUB]=sub_list
        node_list.append(node)

        for i in range(num_topics):
            node = [[],[]]
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
            node = [[],[]]
            pub_list = []
            sub_list = []
            for i in range (num_topics):
                pub_list.append(str(i))
            
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)
        
        # add subscriber nodes
        for s in range(num_subscribers):
            node = [[],[]]
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
            node = [[],[]]
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)
        
        # add subscriber nodes
        node = [[],[]]
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
            node = [[],[]]
            pub_list = []
            sub_list = []
            pub_list.append(str(p))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)
        # subscribers
        print("m " + str(m))
        for s in range(m):
            node = [[],[]]
            pub_list = []
            sub_list = []
            for t in range (num_topics):
                sub_list.append(str(t))
            node[PUB]=pub_list
            node[SUB]=sub_list
            node_list.append(node)    
    else:
        raise ValueError("type unknown. Possible options: [A,B,C,D,E]")


    print("{} {}".format("comm_type:", comm_type))
    process_list = []
    # put them into different processes
    if comm_type == 'intra_process':
        #print("All nodes are in one process: intra-process communication")
        # nothing needs to be done: node_list can be taken as-is
        process_list = [ node_list ]
    elif comm_type == 'dds':
        #print("Every node is executed in a different process: all communication through DDS.")
        process_list=[]
        for node in node_list:
            process_list.append( [node] )
    elif comm_type == 'pub_sub':
        #print("One process for all publisher nodes, one process for all subscriber nodes, i.e. DDS communication between pub and sub.")
        process_list=[[],[]]
        for node in node_list:
            if len(node[PUB])>0 and len(node[SUB])==0:
                process_list[PUB].append(node)
            if len(node[PUB])==0 and len(node[SUB])>0:
                 process_list[SUB].append(node)
            if len(node[PUB])>0 and len(node[SUB])>0:
                raise ValueError("Nodes which both, publish and subscribe topics, are not supported.")
    else:
        raise ValueError('comm_type unknown. Possible options: [intra_process,dds,pub_sub]')

    # debugging output
    #print(process_list)

    # create API command
    cmd_list=[]
    for nodes in process_list:
        cmd = []
        cmd.append (str(len(nodes)))
        for node in nodes:
            #print("{} {}".format("node", node))
            cmd.append("node")
            cmd.append(str(len(node[PUB])))
            cmd.append(str(len(node[SUB])))
            print("{} {} {}".format("node", len(node[PUB]) , len(node[SUB])))
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
        print("args: executor-binary rate[Hz] msg-size topology-type comm-type number-topics \n      [number-subscriber]")
        return

    cmd_name = sys.argv[1] # binary name
    rate =  sys.argv[2]     # in Hz
    msgSize = sys.argv[3]  # in number characters
    topology_type = sys.argv[4] # A - E
    comm_type = sys.argv[5] # dds, intra_process, pub_sub
    num_topics = int(sys.argv[6]) # int
    
    if len(sys.argv[1:])== 7:
        num_subs = int(sys.argv[7])
    else:
        num_subs = 0
    print("{} {} {} {} {} {} {}".format(cmd_name, rate, msgSize, topology_type, comm_type, num_topics, num_subs))

    cmd_list = topology(topology_type, comm_type, num_topics, num_subs)

    #topology('A', 'intra_process', 1, 3)
    #topology('A', 'pub_sub', 1, 3)

    #topology('B', 'dds',3)
    #topology('B', 'intra_process',3)
    #topology('B', 'pub_sub',3)

    #topology('C', 'dds',1, 3)
    #topology('C', 'intra_process',1, 3)
    #topology('C', 'pub_sub',1, 3)

    #topology('D','dds',3)
    #topology('D','intra_process',3)
    #topology('D','pub_sub',3)

    #topology('E','dds', 2,3)
    #topology('E','intra_process', 2,3)
    #topology('E','pub_sub', 2,3)

    # call executor
    for cmd_args in cmd_list:
        my_cmd = []
        my_cmd.append(cmd_name)
        my_cmd.append(rate)
        my_cmd.append(msgSize)
        for arg in cmd_args:
            my_cmd.append(arg)
        subprocess.Popen( my_cmd)

if __name__ == '__main__':
    main()


        

