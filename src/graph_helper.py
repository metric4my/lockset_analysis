#Copyright 2025 Purdue University.
#All rights reserved.


import networkx as nx
from enum import Enum
import matplotlib.pyplot as plt


class GraphHelper:

    def __init__(self):
        self.cbexec_graph = nx.DiGraph()

        self.NType = Enum('NType', ['LAUNCH', 'NODE', 'EXECUTOR', 'CALLBACK_GROUP', 'CALLBACK_FN', 'SPIN', 'FUTURE'])
        self.EType = Enum('EType',['CONTROL', 'CONTAIN', 'INVOKE'])

        self.CbGroup = Enum('CbGroup', ['MUTUALLY_EXCLUSIVE', 'REENTRANT', 'FIFO', 'UNKNOWN'])
        self.CbFunc = Enum('CbFunc', ['SUB', 'SERV', 'CLI', 'ACT_SERV', 'ACT_CLI', 'ACT_SUB', 'TIMER', 'OTHER'])
        self.ThrCount = Enum('ThrCount', ['CORES', 'UNKNOWN'], start=99) # assign large value so that it does not match with numerical value of number of threads
        self.ThrId = Enum('ThrId', ['CORES', 'UNKNOWN'])
        self.SpinType = Enum('SpinType', ['INFINITE', 'ONE', 'AVAILABLE', 'AVAILABLE_LOOP', 'FUTURE'])

        self.id_cnt = 0 #SUFFIX TO BUILD UNIQUE NAME FOR GRAPH NODE
        self.thr_id_cnt = 0 #SUFFIX TO 

        self.node_to_default_dict = {} #NODE TO DEFAULT CALLBACK GROUP AND DEFAULT EXECUTOR DICT
        self.potential_nodes = set() ##TODO check where and when to reset this

        self.thr_dict_list = []
        self.thr_dict = {} #Current: Track THREAD ID TO THREAD FUNCTION NAME for post-processing
        self.func_to_grnode_list = [] #LIST of (CALLING_FUNCTION_LIST, (GRAPH NODES))

        self.cb_dict_list = []
        self.cb_dict = {} #Current: Track STRING TO CALLBACK FUNCTION e.g. topic name to subscriber callback for post-processing
                          #NOTE: callback function name is same as graph node
        self.func_to_string_list = [] #LIST OF (CALLING_CALLBACK_FUNCTION, STRING)

        self.Version = Enum('Version', ['ROSCPP', 'ROSPY', 'RCLCPP', 'RCLPY'])
        self.current_version = None


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # GRAPH FUNCTIONS: SET
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def set_current_version(self, ver):
        self.current_version = ver


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # GRAPH FUNCTIONS: SET
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ''' return: updated graph node name 
        update: add existing node updates the node data 
        '''
    def add_graph_node(self, gr_node, n_type, n_attribs={}, add_suffix=False):
        if gr_node:
            if (n_type in [self.NType.EXECUTOR, self.NType.CALLBACK_GROUP, self.NType.SPIN, self.NType.FUTURE]) or add_suffix:
                self.id_cnt += 1
                gr_node = gr_node + '_' + str(self.id_cnt)
            self.cbexec_graph.add_node(gr_node, type=n_type, ver=self.current_version, **n_attribs)
        return gr_node


    ''' update: add edge creates node if does not exist 
        '''
    def add_graph_edge(self, gr_from_node, gr_to_node, e_type, e_attribs={}):
        if gr_from_node and gr_to_node:
            self.cbexec_graph.add_edge(gr_from_node, gr_to_node, type=e_type, **e_attribs)


    ''' do not copy the graph, relable in place
        update the node to default dict to replace prev node id with the new one
        '''
    def relabel_graph_node(self, prev_id, new_id):
        mapping = {prev_id: new_id}
        self.cbexec_graph = nx.relabel_nodes(self.cbexec_graph, mapping, copy=False)

        if self.node_to_default_dict.get(prev_id):
            self.node_to_default_dict[new_id] = self.node_to_default_dict.pop(prev_id)


    def set_graph_attribute(self, gr_node, n_attrib, val):
        nx.set_node_attributes(self.cbexec_graph, {gr_node:val}, n_attrib)


    # we associate callback groups to the executor with an edge from executor to callback group,
    # because e.g. in rclcpp if 'add'=False for a callback group that group is not added to executor.
    # So, to avoid having association of all callback groups of node to the executor,
    # the edge between node and executor is from node to executor instead from executor to node.
    def set_executor_callback_groups(self):
        node_list = self.get_ros_nodes_all()
        for node_id in node_list:
            cbg_list = self.get_callback_groups_of_node(node_id)
            exec_list = self.get_executor_of_node(node_id)
            for exec_id in exec_list:
                for cbg_id in cbg_list:
                    self.add_graph_edge(exec_id, cbg_id, self.EType.CONTAIN)


    def mark_graph_node_as_local(self, gr_node):
        if self.cbexec_graph.has_node(gr_node):
            self.set_graph_attribute(gr_node, 'local', True)
        ##TODO check for else case?


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # GRAPH FUNCTIONS: GET
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_callback_groups_of_node(self, node_id):
        cbg_list = []
        for n in self.cbexec_graph.neighbors(node_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.CALLBACK_GROUP and self.cbexec_graph.nodes[n].get('add', False):
                ##TODO check if ros1 needs 'add' attribute
                cbg_list.append(n)
        return cbg_list


    def get_executor_of_node(self, node_id):
        exec_list = []
        for n in self.cbexec_graph.neighbors(node_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.EXECUTOR:
                exec_list.append(n)
        return exec_list
    

    def get_invoke_callback_pairs(self):
        cb_nodes = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.CALLBACK_FN]
        cb_edges = [(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=cb_nodes) if val==self.EType.INVOKE]
        return cb_edges
    

    def get_callbacks_of_callback_group(self, cbg_id):
        cb_list = []
        for n in self.cbexec_graph.successors(cbg_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.CALLBACK_FN:
                cb_list.append(n)
        return cb_list
    

    def get_callback_group_of_callback(self, cb_id):
        cbg_list = []
        for n in self.cbexec_graph.predecessors(cb_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.CALLBACK_GROUP:
                cbg_list.append(n)
        return cbg_list
    

    def get_node_of_callback(self, cb_id):
        cbg_list = self.get_callback_group_of_callback(cb_id)
        node_list = [u for u,v,val in self.cbexec_graph.in_edges.data('type', nbunch=cbg_list) \
                            if self.get_attrib_val(u, 'type')==self.NType.NODE]
        return node_list
    

    def get_executor_of_callback_group(self, cbg_id):
        exec_list = []
        for n in self.cbexec_graph.predecessors(cbg_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.EXECUTOR:
                exec_list.append(n)
        return exec_list
    

    def get_executors_all(self):
        exec_list = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.EXECUTOR]
        return exec_list


    def get_callback_groups_all(self):
        cbg_list = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.CALLBACK_GROUP]
        return cbg_list
    

    def get_ros_nodes_all(self):
        node_list = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.NODE]
        return node_list
    

    def get_spins_all(self):
        spin_list = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.SPIN]
        return spin_list
    

    def get_executor_of_spin(self, spin_id):
        exec_list = []
        for n in self.cbexec_graph.successors(spin_id):
            if self.cbexec_graph.nodes[n]['type'] == self.NType.EXECUTOR:
                exec_list.append(n)
        return exec_list
    

    def get_spin_invokes_of_executor(self, exec_id):
        spin_list = [u for u,v,val in self.cbexec_graph.in_edges.data('type', nbunch=exec_id) if val==self.EType.INVOKE]
        return spin_list
    

    def get_spin_invokes_of_spin(self, spin_id):
        spin_list = [u for u,v,val in self.cbexec_graph.in_edges.data('type', nbunch=spin_id) if val==self.EType.INVOKE and \
                            self.get_attrib_val(u, 'type')==self.NType.SPIN]
        return spin_list
    

    # Note: edges is out_edges; returns edges from nodes in nbunch; So, v can be a node of different type.
    def get_invoke_spin_pairs(self):
        spin_nodes = [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.SPIN]
        spin_edges = [(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=spin_nodes) \
                            if val==self.EType.INVOKE and self.get_attrib_val(v, 'type')==self.NType.SPIN]
        return spin_edges
    

    # Note: edges is out_edges; returns edges from nodes in nbunch; So, v can be a node of different type.
    def get_invoke_spin_pairs_of_node(self, node_id):
        exec_list = self.get_executor_of_node(node_id)
        spin_list = []
        for exec_id in exec_list:
            spin_list.extend(self.get_spin_invokes_of_executor(exec_id))
        spin_edges = [(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=spin_list) \
                            if val==self.EType.INVOKE and self.get_attrib_val(v, 'type')==self.NType.SPIN]
        return spin_edges
    
    
    # Note: edges is out_edges; returns edges from nodes in nbunch; So, v can be a node of different type.
    def get_invoke_spin_pairs_from_spin_list(self, spin_list):
        spin_edges = [(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=spin_list) \
                            if val==self.EType.INVOKE and self.get_attrib_val(v, 'type')==self.NType.SPIN]
        return spin_edges
    

    def get_cycles(self):
        return list(nx.simple_cycles(self.cbexec_graph))
    

    # this modifies the original graph
    def get_dfs_cycles(self):
        G = self.cbexec_graph
        cycles_list = []

        # TRY FINDING A DAG.  IF NOT, REMOVE EDGES AND TRY AGAIN. 
        super_break = 0
        while nx.number_of_nodes(G) > 0 and super_break < 1000:
            super_break +=1 
            if not nx.is_directed_acyclic_graph(G):
                try:
                    # SEARCH FOR CYCLE AND REMOVE EDGES
                    edges = nx.find_cycle(G)
                    G.remove_edges_from(edges)

                    cycle = set()
                    for e in edges:
                        cycle.add(e[0])
                        cycle.add(e[1])
                    if cycle:
                        cycles_list.append(list(cycle))

                except:
                    # GIVE UP
                    break
            else:
                # WE HAVE A DIGRAPH
                break

        return cycles_list
    

    def get_graph_nodes_with_local_scope(self):
        gr_node_list = [n for n,val in self.cbexec_graph.nodes.data('local') if val==True]
        return gr_node_list


    def get_attrib_val(self, grnode_id, attrib):
        return self.cbexec_graph.nodes[grnode_id][attrib] if attrib in self.cbexec_graph.nodes[grnode_id] else None
    

    def has_incoming_edge(self, grnode_id):
        return self.cbexec_graph.in_degree(grnode_id) > 0
    

    def has_outgoing_edge(self, grnode_id):
        return self.cbexec_graph.out_degree(grnode_id) > 0
    

    def has_edge(self, u, v):
        return self.cbexec_graph.has_edge(u, v)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # ROS NODE AND ITS DEFAULTS HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def update_node_to_default_cbgroup(self, node_id, default_cbg_id):
        if not self.node_to_default_dict.get(node_id):
            self.node_to_default_dict[node_id] = {'cbg': default_cbg_id}
        else:
            self.node_to_default_dict[node_id]['cbg'] = default_cbg_id


    def setup_node_to_default(self, node_id, cbg_type):
        update = False

        # ADD NODE TO GRAPH IF NEEDED
        if not self.node_to_default_dict.get(node_id):
            if node_id in self.potential_nodes:
                is_active = True
                self.potential_nodes.discard(node_id)
            else:
                is_active = False
            self.add_graph_node(node_id, self.NType.NODE, {'is_active': is_active})
            update = True
        elif not self.node_to_default_dict[node_id].get('cbg'):
            update = True

        if update and cbg_type:
            cbg_id = self.add_graph_node('default_cbg', self.NType.CALLBACK_GROUP, {'group_type': cbg_type, 'add': True})
            self.add_graph_edge(node_id, cbg_id, self.EType.CONTAIN)
            # STORE NODE TO DEFAULT CALLBACK GROUP MAPPING TO FIND THE GROUP FOR THE NODE WHILE ADDING CALLBACKS
            self.update_node_to_default_cbgroup(node_id, cbg_id)


    def get_default_cbgroup_of_node(self, node_id, setup=True):
        if setup:
            self.setup_node_to_default(node_id, self.CbGroup.MUTUALLY_EXCLUSIVE)
        return self.node_to_default_dict[node_id].get('cbg')        
    

    # ROS2
    ##TODO refactor code to remove setup from below functions; handle setup via get node similar to ros1

    # RCLPY
    def get_rate_cbgroup_of_node(self, node_id):
        # node_id is already added to node_to_default_dict while adding default callback group during node setup
        rate_cbg_id = self.node_to_default_dict[node_id].get('rate_cbg')

        if not rate_cbg_id:
            # INITIALIZE REENTRANT CALLBACK GROUP FOR RATE TIMERS
            rate_cbg_id = self.add_graph_node('rate_cbg', self.NType.CALLBACK_GROUP, {'group_type': self.CbGroup.REENTRANT, 'add': True})
            self.add_graph_edge(node_id, rate_cbg_id, self.EType.CONTAIN)
            # STORE NODE TO RATE CALLBACK GROUP MAPPING TO FIND THE GROUP FOR THE NODE WHILE ADDING RATE
            self.node_to_default_dict[node_id]['rate_cbg'] = rate_cbg_id

        return rate_cbg_id

    
    ''' implicit refers to internally created executor
        '''
    def update_node_to_implicit_executor(self, node_id, implicit_exec_id, setup=True):
        if setup:
            self.setup_node_to_default(node_id, self.CbGroup.MUTUALLY_EXCLUSIVE)

        if not self.node_to_default_dict[node_id].get('exec'):
            self.node_to_default_dict[node_id]['exec'] = [implicit_exec_id]
        else:
            self.node_to_default_dict[node_id]['exec'].append(implicit_exec_id)


    def update_node(self, node_id, is_active=False, setup=True):
        if setup:
            self.setup_node_to_default(node_id, self.CbGroup.MUTUALLY_EXCLUSIVE)

        if is_active:
            if self.node_to_default_dict.get(node_id):
                self.set_graph_attribute(node_id, 'is_active', True)
            else:
                self.potential_nodes.add(node_id)


    # ROS1
    def setup_and_get_node(self, node_id=None, def_cbgroup=True):
        if not node_id:
            self.id_cnt += 1
            node_id = 'node_' + str(self.id_cnt)

        if def_cbgroup:
            self.setup_node_to_default(node_id, self.CbGroup.FIFO)
        else:
            self.setup_node_to_default(node_id, cbg_type=None)

        return node_id
    

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # THREAD HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def track_thread_items(self, calling_funcs, items):
        self.func_to_grnode_list.append((calling_funcs, items))


    def track_thread_function(self, thr_func, thr_id):
        self.thr_dict[thr_id] = thr_func


    def get_current_thread_function_dict(self):
        return self.thr_dict
    

    def reset_thread_function_dict(self):
        self.thr_dict = {}

    # NOT USED
    def save_thread_function_dict(self):
        self.thr_dict_list.append(self.thr_dict)


    def new_thread_id(self):
        self.thr_id_cnt += 1
        return self.thr_id_cnt


    ##TODO support lambda functions
    def setup_thread(self, thr_count, setup_last_thr_id=True):
        if thr_count == self.ThrCount.UNKNOWN:
            return self.ThrId.UNKNOWN
        elif thr_count == self.ThrCount.CORES:
            last_thr_id = self.new_thread_id() if setup_last_thr_id else None
            return (last_thr_id , self.ThrId.CORES)
        elif thr_count <= 0:
            return None ##TODO return another value to distinguish between none for current thread and none for invalid id
        elif thr_count == 1:
            last_thr_id = self.new_thread_id() if setup_last_thr_id else None
            return last_thr_id
        else:
            thr_id = []
            for i in range(thr_count-1):
                self.thr_id_cnt += 1
                thr_id.append(self.thr_id_cnt)
            last_thr_id = self.new_thread_id() if setup_last_thr_id else None
            thr_id.append(last_thr_id)
            return thr_id
            
        # TUPLE VALUE (i,n) REPRESENTS n THREADS WHERE THE LAST THREAD ID IS i
        # LIST VALUE [] REPRESENTS MULTIPLE THREADS WITH IDS; 
        # -- use the list representation because one of the thread has id of spin call, 
        # -- which may not be consecutive number to other thread ids, hence tuple can not be used to show the range
        # NUMBER VALUE REPRESENTS A SINGLE THREAD WITH ID
        

    ##TODO use list notation in all situations??
    def get_updated_thread_id_for_graph_node(self, old_thr_id, new_thr_id):
        if not old_thr_id:
            return new_thr_id
        elif isinstance(old_thr_id, list):
            if not old_thr_id[-1]:
                old_thr_id[-1] = new_thr_id
            return old_thr_id
        elif isinstance(old_thr_id, tuple):
            return (new_thr_id, old_thr_id[1]) if not old_thr_id[0] else old_thr_id
        else: # single non-none id
            return old_thr_id
        

    ##TODO support threads across files
    def find_thread_id(self, func):
        (function_obj, name) = func
        thr_ids = [] #There can be multiple thread ids associated with a function, when multiple threads are started with the same function as entry point
        is_thr = False

        for thr_id, thr_func in self.thr_dict.items():
            (thr_function_obj, thr_name) = thr_func
            if name == thr_name:
                is_thr = True
            if function_obj and thr_function_obj:
                # python: scopes_manager
                if isinstance(function_obj, dict) and isinstance(thr_function_obj, dict):
                    if function_obj['sscope'].get_id() != thr_function_obj['sscope'].get_id():
                        is_thr = False
                else: # cpp: functions_manager
                    if function_obj.Id != thr_function_obj.Id:
                        is_thr = False
            if is_thr:
                thr_ids.append(thr_id)
        
        return thr_ids[0] if len(thr_ids) == 1 else thr_ids


    def assign_threads(self):
        for (calling_funcs, gr_nodes) in self.func_to_grnode_list:
            thr_id = None
            for func in calling_funcs:
                thr_id = self.find_thread_id(func)
                if thr_id:
                    ##TODO support updating of thread id for the multiple assignment case: may be append to list?
                    for gr_node in gr_nodes:
                        if self.cbexec_graph.has_node(gr_node):
                            updated_thr_id = self.get_updated_thread_id_for_graph_node(self.cbexec_graph.nodes[gr_node].get('thr_id'), thr_id)
                            self.cbexec_graph.nodes[gr_node]['thr_id'] = updated_thr_id


    # thr_id = single id (not a list or a tuple)?
    def assign_thread_id(self, gr_nodes, thr_id):
        for gr_node in gr_nodes:
            if self.cbexec_graph.has_node(gr_node):
                old_thr_id = self.cbexec_graph.nodes[gr_node].get('thr_id')
                if old_thr_id:
                    thr_id = old_thr_id.append(thr_id) if isinstance(old_thr_id, list) else [old_thr_id, thr_id]
                self.cbexec_graph.nodes[gr_node]['thr_id'] = thr_id


    def find_and_update_callback_spin_links(self):
        link_dict = {}
        for (calling_funcs, gr_nodes) in self.func_to_grnode_list:
            spin_id = None
            exec_id = None
            for gr_node in gr_nodes:
                if self.cbexec_graph.has_node(gr_node):
                    if self.get_attrib_val(gr_node, 'type') == self.NType.SPIN:
                        spin_id = gr_node
                    elif self.get_attrib_val(gr_node, 'type') == self.NType.EXECUTOR:
                        exec_id = gr_node

            if not spin_id:
                continue

            # to support rospy threading model
            has_spin_invoke = self.has_edge(spin_id, exec_id)

            for func in calling_funcs:
                is_cb = self.is_callback_function(func)
                if is_cb:
                    # add invocation edge from callback to spin; name is same as callback node id
                    (function_obj, name) = func
                    if self.cbexec_graph.has_node(name):
                        self.add_graph_edge(name, spin_id, self.EType.INVOKE)

                        # spin node can be associated with only one callback function node
                        link_dict[spin_id] = name

                        if not has_spin_invoke:
                            cb_id = name
                            # there can be be only one cbg and one exec in this case, 
                            # as seaprate thread is invoked for each callback object like subscriber in rospy
                            cbg_id = self.get_callback_group_of_callback(cb_id)[0]
                            exec_id = self.get_executor_of_callback_group(cbg_id)[0]
                            # exec_id is overwritten to add an edge to the executor of the callback that has spin call
                
            if not has_spin_invoke:
                # add edge to the dummy_exec or the executor of the callback that has spin call
                #(there can be no edge only in case of rospy with dummy_exec)
                self.add_graph_edge(spin_id, exec_id, self.EType.INVOKE)

        for r_spin_id, r_cb_id in link_dict.items():
            r_cbg_set = set(self.get_callback_group_of_callback(r_cb_id))

            r_exec_set = set()
            for cbg_id in r_cbg_set:
                r_exec_set.update(self.get_executor_of_callback_group(cbg_id))

            spin_set = set()
            for exec_id in r_exec_set:
                spin_set.update(self.get_spin_invokes_of_executor(exec_id))

            for spin_id in spin_set:
                if spin_id == r_spin_id:
                    continue
                # another spin function from some callback
                if spin_id in link_dict:
                    # check if their callbacks belong to same callback group
                    cb_id = link_dict[spin_id]
                    cbg_set = set(self.get_callback_group_of_callback(cb_id))
                    cbg_intersect_len = len(r_cbg_set.intersection(cbg_set))
                    if cbg_intersect_len==len(r_cbg_set) and cbg_intersect_len==len(cbg_set):
                        # if more than one common callback groups, they may be two separate mutually exclusive groups that can execute concurrently.
                        if (cbg_intersect_len == 1):
                            if (self.get_attrib_val(min(cbg_set), 'group_type') == self.CbGroup.MUTUALLY_EXCLUSIVE):
                                continue
                            elif (self.get_attrib_val(min(cbg_set), 'group_type') == self.CbGroup.FIFO) and \
                                     (cb_id == r_cb_id) and self.get_attrib_val(cb_id, 'concurrent') == False:
                                continue
                # other spin is not from a callback or is from a callback with different group or reentrant or fifo group; so add an invocation edge
                self.add_graph_edge(spin_id, r_spin_id, self.EType.INVOKE)
                # disable the below if we find cycles using get_dfs_cycles for graph checker
                # remove the edge from callback to spin if there are other incoming edges from spin noded to this spin 
                # if self.cbexec_graph.has_edge(r_cb_id, r_spin_id):
                #    self.cbexec_graph.remove_edge(r_cb_id, r_spin_id)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # CALLBACK INVOCATION HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def track_potential_callback_link(self, calling_cb, id_string):
        self.func_to_string_list.append((calling_cb, id_string))
    

    def track_callback_function(self, cb, id_string):
        self.cb_dict[id_string] = cb


    def is_callback_function(self, func):
        (function_obj, name) = func
        is_cb = False

        for id_string, cb in self.cb_dict.items():
            (cb_function_obj, cb_name) = cb
            if name == cb_name:
                is_cb = True
            if function_obj and cb_function_obj:
                 # python: scopes_manager
                if isinstance(function_obj, dict) and isinstance(cb_function_obj, dict):
                    if function_obj['sscope'].get_id() != cb_function_obj['sscope'].get_id():
                        is_cb = False
                else: # cpp: functions_manager
                    if function_obj.Id != cb_function_obj.Id:
                        is_cb = False
            if is_cb:
                break

        return is_cb
            

    def get_current_callback_function_dict(self):
        return self.cb_dict
    

    def reset_callback_function_dict(self):
        self.cb_dict = {}


    def save_callback_function_dict(self):
        self.cb_dict_list.append(self.cb_dict)


    def setup_callback_links(self):
        for (func, id_string) in self.func_to_string_list:
            (function_obj, name) = func
            if self.cbexec_graph.has_node(name):
                for cb_dict in self.cb_dict_list:
                    if id_string in cb_dict:
                        cb = cb_dict[id_string]
                        (cb_function_obj, cb_name) = cb
                        if self.cbexec_graph.has_node(cb_name):
                            self.add_graph_edge(name, cb_name, self.EType.INVOKE)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # THREAD GROUPS HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                            
    


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # GRAPH DISPLAY FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def print_graph(self, graph_name='output/cbexec_graph.png'):
        #pos = nx.multipartite_layout(self.cbexec_graph, subset_key='type')
        pos = nx.circular_layout(self.cbexec_graph, scale=2)

        n_color_map = {self.NType.NODE: 'skyblue', 
                    self.NType.EXECUTOR: 'cornflowerblue', 
                    self.NType.SPIN: 'lightgreen',
                    self.NType.CALLBACK_GROUP: 'lightsalmon',
                    self.NType.CALLBACK_FN: 'peachpuff',
                    self.NType.FUTURE: 'peachpuff'}
        
        n_colors = [n_color_map[node[1]['type']] for node in self.cbexec_graph.nodes(data=True)]
        nx.draw_networkx_nodes(self.cbexec_graph, pos, node_color=n_colors, node_size=1000)

        nx.draw_networkx_labels(self.cbexec_graph, pos, font_size=8, verticalalignment='bottom')

        thr_attr_dict = nx.get_node_attributes(self.cbexec_graph, 'thr_id') 
        updated_thr_attr_dict = {}
        for n, attr in thr_attr_dict.items():
            attr_str = 'thread: (' + str(attr[0]) + ', ' + str(attr[1]) + ')' if isinstance(attr, tuple) else 'thread: ' + str(attr)
            updated_thr_attr_dict[n] = attr_str
        thr_attr_pos = {n: (coord[0], coord[1]+0.3) for n, coord in pos.items() if n in thr_attr_dict}
        nx.draw_networkx_labels(self.cbexec_graph, thr_attr_pos, font_size=8, labels=updated_thr_attr_dict)

        e_color_map = {self.EType.CONTAIN: 'darkgray',
                       self.EType.CONTROL: 'blue',
                       self.EType.INVOKE: 'green'}
        
        e_colors = [e_color_map[edge[2]['type']] for edge in self.cbexec_graph.edges(data=True)]
        nx.draw_networkx_edges(self.cbexec_graph, pos, edge_color=e_colors, width=1.0, node_size=1000)

        print("GRAPH: #Nodes: " + str(self.cbexec_graph.number_of_nodes()) + ", #Edges: " + str(self.cbexec_graph.number_of_edges()))
        print('- '*40)
        self.collect_ne_types()

        plt.axis('off')
        plt.savefig(graph_name) 
        plt.close()

    def collect_ne_types(self):
        # [n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.CALLBACK_FN]
        # [(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=cb_nodes) if val==self.EType.INVOKE]
        node_cnt = len([n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.NODE])
        exec_cnt = len([n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.EXECUTOR])
        spin_cnt = len([n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.SPIN])
        cbgp_cnt = len([n for n,val in self.cbexec_graph.nodes.data('type') if val==self.NType.CALLBACK_GROUP])
        cbfn_cnt = len([n for n,val in self.cbexec_graph.nodes.data('type') if (val==self.NType.CALLBACK_FN or val==self.NType.FUTURE)])

        ctrl_cnt = len([(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=[n for n,val in self.cbexec_graph.nodes.data('type')]) if val==self.EType.CONTROL])
        cntn_cnt = len([(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=[n for n,val in self.cbexec_graph.nodes.data('type')]) if val==self.EType.CONTAIN])
        invk_cnt = len([(u, v) for u,v,val in self.cbexec_graph.edges.data('type', nbunch=[n for n,val in self.cbexec_graph.nodes.data('type')]) if val==self.EType.INVOKE])

        print("TY: %d %d %d %d %d %d %d %d\n" % (node_cnt, exec_cnt, spin_cnt, cbgp_cnt, cbfn_cnt, ctrl_cnt, cntn_cnt, invk_cnt))
