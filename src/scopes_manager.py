#Copyright 2025 Purdue University.
#All rights reserved.


from graph_helper import GraphHelper
import symtable
import networkx as nx
import pyan
import os

##TODO add functon call graph edges to this structure?

class ScopesManager:
    
    def __init__(self, gh) -> None:
        self.gh = gh #GraphHelper()
        self.scopes = None
        self.all_scopes = []
        self.function_graph = None


    def find_scopes(self, source):
        self.scopes = {}
        
        st = symtable.symtable(source, "", "exec")
        self.scopes[st.get_id()] = {'stype': st.get_type(),
                                    'sname': st.get_name(),
                                    'sparent': None,
                                    'sscope': st,
                                    'function_graph_edges': []}

        st_list = [st]
        while len(st_list) > 0:
            st_current = st_list.pop()
            if st_current.has_children():
                st_children = st_current.get_children()
                for st_child in st_children:
                    # add class name to function name
                    name = ''
                    if st_current.get_type() == 'class' and st_child.get_type() == 'function':
                        name = name + st_current.get_name() + '::'
                    name = name + st_child.get_name()

                    self.scopes[st_child.get_id()] = {'stype': st_child.get_type(),
                                                    'sname': name,
                                                    'sparent': st_current,
                                                    'sscope': st_child,
                                                    'function_graph_edges': []}
                    st_list.append(st_child)


    def get_type(self, scope):
        return scope.get('stype') if scope else None
    

    def get_name(self, scope):
        return scope.get('sname') if scope else None
    

    def get_parent(self, scope):
        parent_scope = None
        if scope:
            parent = scope.get('sparent')
            if parent:
                parent_scope = self.scopes.get(parent.get_id())
        return parent_scope
    

    def get_scope(self, scope_type, scope_name):
        for scope_dict in self.scopes.values():
            if scope_dict['stype'] == scope_type and scope_dict['sname'] == scope_name:
                return scope_dict
        return None
    

    def lookup_name_in_scope(self, name, scope):
        st = scope['sscope']
        return st.lookup(name) if name in st.get_identifiers() else None
        
        
    def build_function_graph(self, source):
        # BUILD CALL GRAPH
        self.function_graph = nx.DiGraph()
        G = self.function_graph

        for k, scope_dict in self.scopes.items():
            if scope_dict['stype'] in ['module', 'function']:
                node = k
                G.add_node(node)
    
        self.add_edges_to_function_graph(G, source)

        self.function_graph = G

        #self.debug_print_function_graph()
        

    def add_edges_to_function_graph(self, G, source):
        cgtemp_filename = 'output/stream.tgf'
        pyan.main(['pyan3', source, '--tgf', '--annotated', '--file', cgtemp_filename])

        if os.path.exists(cgtemp_filename):
            cgtemp_nodes_dict = {}

            f = open(cgtemp_filename)
            for line in f:
                line = line.rstrip()
                if line == '#':
                    continue
                parts = line.split(' ')

                number_of_parts = len(parts)
                if number_of_parts == 2:
                    cgtemp_nodes_dict[parts[0]] = (parts[1], 'module')
                elif number_of_parts == 4:
                    second_parts = parts[1].split('\\n')

                    name = ''
                    if second_parts[3] == 'method' and '.' in parts[3]:
                        fourth_parts = parts[3].split('.')
                        if fourth_parts[-1].endswith(')'):
                            class_name = fourth_parts[-1].replace(')', '') 
                            name = name + class_name + '::'
                    name = name + second_parts[0]

                    cgtemp_nodes_dict[parts[0]] = (name, second_parts[3])
                elif number_of_parts == 3:
                    from_scope = None
                    to_scope = None

                    from_node = parts[0]
                    from_name, from_type = cgtemp_nodes_dict.get(from_node, (None, None))

                    if from_name and from_type:
                        if from_type == 'method' or from_type == 'function':
                            # symtable uses 'function term, whereas pyan uses 'method' term for class function and 'function' term for module function
                            from_scope = self.get_scope('function', from_name)
                        elif from_type == 'module':
                            # symtable uses 'top' term, whereas pyan uses module nam i.e. file name
                            from_scope = self.get_scope(from_type, 'top')

                    if not from_scope:
                        continue

                    to_node = parts[1]
                    to_name, to_type = cgtemp_nodes_dict.get(to_node, (None, None))

                    if to_name and to_type:
                        if to_type == 'method' or to_type == 'function':
                            # symtable uses 'function term, whereas pyan uses 'method' term for class function and 'function' term for module function
                            to_scope = self.get_scope('function', to_name)
                        elif to_type == 'module':
                            # symtable uses 'top' term, whereas pyan uses module nam i.e. file name
                            to_scope = self.get_scope(to_type, 'top')

                    if not to_scope:
                        continue

                    if not G.has_edge(from_scope['sscope'].get_id(), to_scope['sscope'].get_id()):
                        G.add_edge(from_scope['sscope'].get_id(), to_scope['sscope'].get_id())
                        from_scope['function_graph_edges'].append(to_scope)


    def filter_function_graph(self, init_func_scopes):
        # REMOVE EDGES TO CALLBACK FUNCTIONS - pyan adds edge to callback function e.g. subscriber def with callback function
        # REMOVE EDGES TO __init__ - pyan adds edge to __init__ even when no object is created but just 'self.<>' is referenced
        G = self.function_graph

        edges_to_delete = []
        for from_node, to_node in G.edges:
            to_scope = self.scopes.get(to_node)
            if to_scope and to_scope['stype'] == 'function':
                if self.gh.is_callback_function((to_scope, to_scope['sname'])):
                    edges_to_delete.append((from_node, to_node))
                elif to_scope['sname'].endswith('__init__'):
                    from_scope = self.scopes.get(from_node)
                    if from_scope and from_scope['stype'] == 'function' and (from_scope['sname'] not in init_func_scopes):
                        edges_to_delete.append((from_node, to_node))

        for from_node, to_node in edges_to_delete:
            if G.has_edge(from_node, to_node):
                G.remove_edge(from_node, to_node)
                from_scope = self.scopes.get(from_node)
                to_scope = self.scopes.get(to_node)
                if from_scope:
                    from_scope['function_graph_edges'].remove(to_scope)

        self.function_graph = G

        #print('AFTER FILTER:')
        #self.debug_print_function_graph()


    def get_calling_functions_list(self, function_scope):
        if not self.function_graph or not function_scope:
            return []

        # REVERSE THE FUNCTION GRAPH SO THAT EDGE GOES FROM CALLED FUNCTION TO CALLER FUNCTION
        # THEN, DFS FROM THE CALLED FUNCTION GIVES THE CHAIN OF CALLER FUNCTIONS 
        reverse_function_graph = self.function_graph.reverse(copy=False)
        node_list = list(nx.dfs_preorder_nodes(reverse_function_graph, source=function_scope['sscope'].get_id()))
        if not node_list:
            return []
        
        calling_func_list = []

        for node in node_list:
            function_obj = self.scopes[node]
            # parent_obj = self.scopes.get(function_obj['sparent'].get_id()) if function_obj['sparent'] else None
            # name = ''
            # if parent_obj and parent_obj['stype'] == 'class':
            #     name = name + parent_obj['sname'] + '::'
            # name = name + function_obj['sname']
            name = function_obj['sname']
            calling_func_list.append((function_obj, name))

        assert(len(node_list) == len(calling_func_list))

        return calling_func_list
    

    def debug_print_function_graph(self):
        for scope_dict in self.scopes.values():
            print("%s :" % scope_dict['sname'])
            for edge in scope_dict['function_graph_edges']:
                print(' --> %s' % edge['sname'])
    
