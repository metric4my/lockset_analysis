#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


import cppcheckdata
import networkx as nx
from collections import OrderedDict, defaultdict


class FunctionsManager:

    def __init__(self):
        self.function_graph = None # current graph; temp variable that holds graph for the dump under process
        self.all_function_graphs = []


    def find_functions(self, a_cppcheck_configuration):
        ''' LINEAR SCAN THROUGH TOKENS TO FIND 'function' TOKENS THAT HAVE GLOBAL SCOPE.
            COLLECT AND RETURN DICT CONTAINING FUNCTION START AND END TOKEN POINTERS 
            output: dict containing function start and end tokens
            '''
        function_dicts = {}

        # FIND FUNCTIONS IN 'SCOPES' REGION OF DUMP FILE, START AND END TOKENs
        for s in a_cppcheck_configuration.scopes:
            if s.type=='Function': 
                # SCAN ALL FUNCTIONS UNLESS LIST OF FUNCTIONS SPECIFIED
                function_dicts[s.Id] = {'name': s.className,
                                        'linenr': s.bodyStart.linenr,
                                        'tokenStart': s.bodyStart, 
                                        'tokenEnd': s.bodyEnd, 
                                        'scopeObject': s,
                                        'symbol_table': {},
                                        'function_graph_edges': [],
                                        'function': s.function}
                # CONSTRUCT LIST OF ROOT TOKENS
                function_dicts[s.Id]['root_tokens'] = self.find_root_tokens(s.bodyStart, s.bodyEnd)
                    
        #print "Found %d functions..." % len(function_dicts)
        
        return function_dicts
    

    def find_root_tokens(self, tokenStart, tokenEnd):
        ''' FOR A FUNCTION DEFINED AS ALL TOKENS FROM tokenStart TO tokenEnd, FIND THE ROOTS.
            input: tokenStart, a CPPCheckData Token, first token in a function
            input: tokenEnd, a CPPCheckData Token, last token in a function
            output: a list of root_tokens, in flow order
            '''
        root_tokens_set = set()

        current_token = tokenStart
        while(current_token != tokenEnd):  #TODO: reverse token set exploration to top-down instead of bottom-up
            # HAS A PARENT
            if current_token.astParent: 
                a_parent = current_token.astParent
                has_parent = True
                while has_parent:
                    # HAS NO PARENT, THEREFORE IS ROOT
                    if not a_parent.astParent:     
                        root_tokens_set.add(a_parent)
                        a_parent.isRoot = True  
                        has_parent = False
                    else:
                        a_parent = a_parent.astParent 
            current_token = current_token.next

        
        root_tokens = list(root_tokens_set)
        # SORT NUMERICALLY BY LINE NUMBER
        root_tokens = sorted(root_tokens, key=lambda x : int(x.linenr))

        return root_tokens
    

    def build_function_graph(self, analysis_dict):
        ''' BUILDS DIRECTED FUNCTION GRAPH.
            input:  a dictionary of functions from this dump file
            output: none 
            side effect: creates a graph linked to this object
            '''
        # BUILD CALL GRAPH
        self.function_graph = nx.DiGraph()
        G = self.function_graph

        for k, function_dict in analysis_dict.items():
            if function_dict['function']:
                node = function_dict['function'].Id
                G.add_node(node)
                all_attr = nx.get_node_attributes(G, 'function_id') # function_id == function_scope_id
                all_attr[node] = k
                nx.set_node_attributes(G, all_attr, 'function_id')
                self.add_edges_to_function_graph(function_dict, G, node)

        self.function_graph = G


    def add_edges_to_function_graph(self, function_dict, G, current_node):
        current_token = function_dict['tokenStart']
        end_token = function_dict['tokenEnd']

        # TERMINATION GUARENTEED IF DUMP FILE IS WELL FORMED
        while current_token is not end_token: 
            # ON FIRST LOOP, SKIP SELF-REFERENCE
            current_token = current_token.next  
            if current_token.function and self.is_called_function_token(current_token):
                if not G.has_edge(current_node, current_token.function.Id):
                    G.add_edge(current_node, current_token.function.Id)
                    function_dict['function_graph_edges'].append(current_token.function)


    @staticmethod
    def is_called_function_token(token):
        """Distinguish a call target from a reference passed as an argument."""
        callee = token
        while (callee.astParent and callee.astParent.str in ('.', '::')
               and callee.astParent.astOperand2 is callee):
            callee = callee.astParent
        call = callee.astParent
        return bool(call and call.str == '(' and call.astOperand1 is callee)


    def make_sorted_analysis_dict_from_function_graph(self, analysis_dict):
        ''' BUILDS A TOPO SORTED FUNCTION GRAPH. 
            THIS ALLOWS THE ANALYSIS TO START ON FUNCTION LEAFS, SO WE CAN HOPEFULLY DISCOVER TYPES OF THE RETURN VALUE.  
            THE FUNCTION GRAPH MAY HAVE CYCLES (recursion, for example), THEREFORE WE REMOVE THESE EDGES FROM THE GRAPH
            AND ANALYZE THEM LAST (<-- not sure this is best)
            input: a dictionary of functions from this dump file
            output: OrderedDict of functions
            postcondition: returned dict must be the same length as the input dict, and contain all the same elements
            '''
        return_dict = OrderedDict()
        G = self.function_graph 

        # TRY FINDING A DAG.  IF NOT, REMOVE EDGES AND TRY AGAIN. 
        super_break = 0
        while nx.number_of_nodes(G) > 0 and super_break < 1000:
            super_break +=1 
            if not nx.is_directed_acyclic_graph(G):
                try:
                    # SEARCH FOR CYCLE AND REMOVE EDGES
                    edges = nx.find_cycle(G)
                    G.remove_edges_from(edges)
                    print('Function graph has cycle %s' % edges)
                except:
                    print('Function graph is not a DAG and does not have a cycle!')
                    # GIVE UP AND RETURN UNSORTED 
                    return analysis_dict
            else:
                # WE HAVE A DIGRAPH, CAN PROCEED ( and topo sort )
                break
        
        if nx.number_of_nodes(G) == 0:
            # RETURN UNCHANGED
            return analysis_dict
        
        # GET THE ATTRIBUTE OF ALL NODES: USED FOR ACCESSING DETAILS OF A NODE
        function_id_attr_dict = nx.get_node_attributes(G, 'function_id')

        # WE HAVE A DIRECTED GRAPH WITH NODES, CAN SORT AND ADD NODES TO ORDERED LIST
        function_graph_topo_sort = list(nx.topological_sort(G))
        #function_graph_topo_sort_reversed = list(function_graph_topo_sort)[::-1]

        ##--------------------------------------
        ##TODO possible solution of order change; remove/replace this section and uncomment above line if does not work
        
        #self.debug_print_node_list(function_graph_topo_sort, analysis_dict, "function_graph_topo_sort")

        root_nodes = []
        for node in function_graph_topo_sort:
            if self.function_graph.in_degree(node) == 0:
                root_nodes.append(node)

        #self.debug_print_node_list(root_nodes, analysis_dict, "root_nodes")

        function_graph_topo_sort_updated = []
        topo_sort_mid_part = []
        topo_sort_last_part = []
        main_node = None
        ctor_node = None
        
        for node in function_graph_topo_sort:
            if node in root_nodes:
                if analysis_dict[function_id_attr_dict[node]]['function'].type == 'Constructor':
                    ctor_node = node
                elif analysis_dict[function_id_attr_dict[node]]['name'] == 'main':
                    main_node = node
                elif ctor_node or main_node:
                    topo_sort_mid_part.append(node)
                else:
                    function_graph_topo_sort_updated.append(node)
            else: # NON-ROOT NODES
                if ctor_node or main_node:
                    topo_sort_last_part.append(node)
                else:
                    function_graph_topo_sort_updated.append(node)

        if main_node:
            topo_sort_mid_part.append(main_node)
        if ctor_node:
            topo_sort_mid_part.append(ctor_node)

        function_graph_topo_sort_updated.extend(topo_sort_mid_part)
        function_graph_topo_sort_updated.extend(topo_sort_last_part)

        #self.debug_print_node_list(function_graph_topo_sort_updated, analysis_dict, "function_graph_topo_sort_updated")

        function_graph_topo_sort_reversed = function_graph_topo_sort_updated[::-1]

        #self.debug_print_node_list(function_graph_topo_sort_reversed, analysis_dict, "function_graph_topo_sort_reversed")

        ##--------------------------------------        

        # CREATE RETURN DICT FROM TOPO SORT
        for node in function_graph_topo_sort_reversed:
            if node in function_id_attr_dict:
                # ADD FUNCTION TO NEW DICTIONARY - THIS IS THE EXPLORE ORDER
                return_dict[function_id_attr_dict[node]] = analysis_dict[function_id_attr_dict[node]]

        ##TODO should we change the order to something else
        ##TODO or process the declarations from all function dics first and then process other api?
        ##TODO or make function graph based on data (ros objects of interest) dependency between functions 
  
        # ADD ANY REMAINING FUNCTIONS NOT IN THE TOPO SORT TO THE ORDERED DICT
        for k in analysis_dict.keys():
            if k not in return_dict:
                return_dict[k] = analysis_dict[k]

        assert (len(return_dict) == len(analysis_dict))

        #self.debug_print_function_graph(return_dict)

        return return_dict
    

    def get_calling_functions_list(self, function, analysis_dict):
        if not self.function_graph or not function:
            return []

        # REVERSE THE FUNCTION GRAPH SO THAT EDGE GOES FROM CALLED FUNCTION TO CALLER FUNCTION
        # THEN, DFS FROM THE CALLED FUNCTION GIVES THE CHAIN OF CALLER FUNCTIONS 
        reverse_function_graph = self.function_graph.reverse(copy=False)
        node_list = list(nx.dfs_preorder_nodes(reverse_function_graph, source=function.Id))
        if not node_list:
            return []
        
        calling_func_list = []

        function_id_attr_dict = nx.get_node_attributes(self.function_graph, 'function_id')
        for node in node_list:
            if node in function_id_attr_dict:
                k = function_id_attr_dict[node]
                function_dict = analysis_dict[k]
                function_obj = function_dict['function']
                func_def_token = function_obj.tokenDef
                name = ''
                # ADD CLASS NAME TO FUNCTION NAME
                if func_def_token.scope and func_def_token.scope.type == 'Class':
                    name = name + func_def_token.scope.className + '::' 
                name = name + function_dict['name']
                calling_func_list.append((function_obj, name))

        assert (len(node_list) == len(calling_func_list))

        return calling_func_list


    def debug_print_function_graph(self, analysis_dict):
        if not analysis_dict:
            return
        for function_dict in analysis_dict.values():
            print("%s :" % function_dict['name'])
            for edge in function_dict['function_graph_edges']:
                print(' --> %s' % edge.name)

    
    def debug_print_node_list(self, function_graph_node_list, analysis_dict, print_title=""):
        print("%s: " % print_title)

        function_id_attr_dict = nx.get_node_attributes(self.function_graph, 'function_id')
        for node in function_graph_node_list:
            function_dict = analysis_dict[function_id_attr_dict[node]]
            print("%s" % function_dict['name'])
