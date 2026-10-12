#Copyright 2025 Purdue University.
#All rights reserved.


from graph_helper import GraphHelper
import py_walker_utils as pwu
import ast


class PY1Walker(ast.NodeVisitor):
    
    def __init__(self, gh, psh) -> None:
        self.gh = gh #GraphHelper()
        self.psh = psh #pwu.PYSymbolHelper()

        self.global_dummy_exec_id = None # for rospy

        self.current_node_id = None
        self.current_scope = [('module', 'top')] # stack of (scope type, scope name) from symtable package
        self.current_class_scope_name = None
        self.other_classes = []

        self.thr_track_tokens = [] # ast nodes tracking the potential processing in a new thread; e.g. spin
        self.cb_link_track_tokens= [] # tokens tracking the potential cb-cb invoke links; e.g. publish, service call, async calls with future
        self.local_track_tokens = [] # tokens tracking the local declrations of node, subscriber, timer, client, server, etc.
        self.spin_locations = [] # functions that call spin
        self.init_func_scopes = [] # function scope names that create objects (invoke __init__)

        ##TODO disabled it by default; should be false like other versions??
        self.disable_local_check = True


    def collect(self, astnode):
        self.setup_main_thread()
        # VISIT AST TREE IN POST ORDER: LEFT-RIGHT-NODE
        self.visit(astnode)


    def visit_Call(self, astnode: ast.Call):
        super().generic_visit(astnode)

        func_name = pwu.get_name(astnode.func)
        (func_obj, func_str) = pwu.get_object_and_name(astnode.func)

        if func_name == 'rospy.init_node':
            node_arg = pwu.get_arg_at_pose_from_left(astnode, 'name', 1)
            node_id = pwu.get_string_value(node_arg)
            if node_id:
                if self.current_node_id and (node_id != self.current_node_id):
                    self.gh.relabel_graph_node(self.current_node_id, node_id)
                else:
                    self.gh.setup_and_get_node(node_id, def_cbgroup=False)

                self.gh.update_node(node_id, is_active=True, setup=False)
                self.current_node_id = node_id

                pwu.set_val(astnode, 'id', node_id)
                pwu.set_val(astnode, 'is_active', True)

                pwu.set_val(astnode.func, 'id', (node_id,))
                self.track_thr_token(astnode.func)

        # in rospy, spin is just serves as a blocking function with sleep.
        # ''' rospy.spin() '''
        # ''' service.spin() '''
        elif func_str == 'spin' and func_obj:
            exec_id = None

            if func_name == 'rospy.spin':
                node_id = self.get_node_val_id()
                if node_id:
                    exec_id = self.get_global_dummy_exec_id()
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                    # do not add edge to the executor yet, as we do not know the spin location (added in post-processing)
            else:
                serv_token = func_obj
                (var_sym, sym_key) = self.get_var_symbol(serv_token)
                if var_sym:
                    exec_id = self.get_sym_val(var_sym, 'exec', sym_key)

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

            if exec_id:      
                pwu.set_val(astnode.func, 'id', (spin_id, exec_id))
                self.track_thr_token(astnode.func)

                self.track_spin_locations(astnode.func)

        elif func_name == 'rospy.Subscriber':
            cb_arg = pwu.get_arg_at_pose_from_left(astnode, '', 3)
            cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
            cb_func = self.psh.get_callback_function(cb_func_name)
            if cb_func:
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SUB})
                self.gh.track_callback_function(cb_func, cb_id)

                cbg_id = self.gh.add_graph_node('sub_cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                exec_id = self.gh.add_graph_node('sub_exec', self.gh.NType.EXECUTOR, {'thr_count': 1, 'dummy': True})
                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

                node_id = self.get_node_val_id()
                self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)
                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                spin_id = self.gh.add_graph_node('dummy_spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE, 'dummy': True})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode, 'id', cb_id)
                
                thr_id = self.gh.setup_thread(1)
                self.gh.assign_thread_id((exec_id,), thr_id)

        # ''' rospy.Timer(period, callback, oneshot=False, reset=False) '''
        elif func_name == 'rospy.Timer':
            cb_arg = pwu.get_arg_at_pose_from_left(astnode, '', 2)
            cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
            cb_func = self.psh.get_callback_function(cb_func_name)
            if cb_func:
                t_arg = pwu.get_arg_at_pose_from_left(astnode, '', 1)
                t = pwu.get_time_value(t_arg)
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.TIMER, 'time': t})
                self.gh.track_callback_function(cb_func, cb_id)

                cbg_id = self.gh.add_graph_node('timer_cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                exec_id = self.gh.add_graph_node('timer_exec', self.gh.NType.EXECUTOR, {'thr_count': 1, 'dummy': True})
                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

                node_id = self.get_node_val_id()
                self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)
                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                spin_id = self.gh.add_graph_node('dummy_spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE, 'dummy': True})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode, 'id', cb_id)
                
                thr_id = self.gh.setup_thread(1)
                self.gh.assign_thread_id((exec_id,), thr_id)

        # ''' rospy.Service(name, service_class, handler, buff_size=65536) '''
        elif func_name == 'rospy.Service':
            cb_arg = pwu.get_arg_at_pose_from_left(astnode, '', 3)
            cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
            cb_func = self.psh.get_callback_function(cb_func_name)
            if cb_func:
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
                self.gh.track_callback_function(cb_func, cb_id)

                cbg_id = self.gh.add_graph_node('serv_cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                exec_id = self.gh.add_graph_node('serv_exec', self.gh.NType.EXECUTOR, {'thr_count': 1, 'dummy': True})
                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

                node_id = self.get_node_val_id()
                self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)
                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                spin_id = self.gh.add_graph_node('dummy_spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE, 'dummy': True})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode, 'id', cb_id)
                pwu.set_val(astnode, 'exec', exec_id)
                
                thr_id = self.gh.setup_thread(1)
                self.gh.assign_thread_id((exec_id,), thr_id)


        elif func_name in self.other_classes:
            # track function scopes that create objects i.e. invoke __init__ method of classes.
            self.track_init_func_scope()


    def visit_Assign(self, astnode: ast.Assign):
        super().generic_visit(astnode)

        if hasattr(astnode.value, 'val'):
            for t in astnode.targets:
                (t_sym, sym_key) = self.get_var_symbol(t)

                if t_sym:
                    for r_key, r_val in astnode.value.val.items():
                        self.update_sym_val(t_sym, r_key, r_val, sym_key)

                        # track local declarations of objects like node, subscriber, timer, client, server, etc.
                        if (not self.disable_local_check) and r_key == 'id' and self.psh.is_local_symbol(t_sym):
                            #self.gh.mark_graph_node_as_local(r_val)
                            self.track_local_token(astnode.value)

            if self.disable_local_check:
                ##TODO disabled it by default; should be false like other versions??
                self.disable_local_check = True


    def visit_FunctionDef(self, astnode: ast.FunctionDef):
        func_name = ''
        if self.current_class_scope_name:
            func_name = func_name + self.current_class_scope_name + '::'
        func_name = func_name + astnode.name

        self.current_scope.append(('function', func_name))
        super().generic_visit(astnode)

        # workaround for self variables
        self.psh.update_current_class_symbols(self.get_current_scope())

        self.current_scope.pop()


    def visit_ClassDef(self, astnode: ast.ClassDef):
        class_name = astnode.name

        self.other_classes.append(class_name)

        self.current_scope.append(('class', class_name))
        self.current_class_scope_name = class_name
        super().generic_visit(astnode)

        # workaround for self variables
        self.psh.update_current_class_symbols(self.get_current_scope())

        self.current_class_scope_name = None
        self.current_scope.pop()

        # workaround for self variables
        self.psh.reset_current_class_symbols()


    def visit_While(self, astnode: ast.While):
        super().generic_visit(astnode)

        if isinstance(astnode.test, ast.UnaryOp) and isinstance(astnode.test.op, ast.Not) and isinstance(astnode.test.operand, ast.Call):
            func_name = pwu.get_name(astnode.test.operand.func)
            if func_name == 'rospy.is_shutdown':
                self.track_spin_locations(astnode)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # RETURNS A TUPLE (SYMBOL, KEY) AS SYMTABLE PACKAGE HAS A SYMBOL FOR E.g. self IN self.var
    # return (SYMBOL(self), 'var') when self.var
    # return (SYMBOL(var), '') when var
    def get_var_symbol(self, astnode):
        (var_obj, var_str) = pwu.get_object_and_name(astnode)
        var_name = pwu.get_name(var_obj) if var_obj else var_str
        current_scope = self.get_current_scope()
        var_sym = self.psh.get_symbol(var_name, current_scope)
        sym_key = var_str if var_obj else ''

        return (var_sym, sym_key)
    

    def get_sym_val(self, sym, val_key, sym_key=''):
        if sym_key:
            val_key = sym_key + '_' + val_key

        return self.psh.get_val(sym, val_key)


    def update_sym_val(self, sym, new_key, new_val, sym_key=''):
        if sym_key:
            new_key = sym_key + '_' + new_key

        self.psh.set_val(sym, new_key, new_val)
    
                
    def get_node_val_id(self):
        ##TODO rospy does not have a node variable, so no node_id is stored in a variable's val directory?
        if not self.current_node_id:
            self.current_node_id = self.gh.setup_and_get_node(node_id=None, def_cbgroup=False)
    
        return self.current_node_id
    

    def get_global_dummy_exec_id(self):
        if not self.global_dummy_exec_id:
            # INITIALIZE GLOBAL EXECUTOR
            self.global_dummy_exec_id = self.gh.add_graph_node('dummy_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
        
        return self.global_dummy_exec_id


    def get_current_scope(self):
        if len(self.current_scope) >= 1:
            return self.current_scope[-1]
    

    def get_main_scope(self):
        if len(self.current_scope) >= 1:
            return self.current_scope[0]


    def get_function_scope_for_token(self, astnode):
        current_scope = self.get_current_scope()
        parent_scope = self.current_scope[-2] if len(self.current_scope) >= 2 else None

        #TODO check if current is a function type and process 'top' as '__main__'
        func_scope = self.psh.get_function_object(current_scope, parent_scope)
        return func_scope
    

    def setup_main_thread(self):
        main_scope = self.get_main_scope()
        func_obj = self.psh.get_function_object(main_scope, parent_info=None)
        thr_func = (func_obj, 'top')
        thr_id = self.gh.setup_thread(1)
        self.gh.track_thread_function(thr_func, thr_id)
    
                
    def track_thr_token(self, astnode):
        # store the function scope of token here, as current scope stack changes while traversing ast
        self.thr_track_tokens.append((self.get_function_scope_for_token(astnode), astnode.val['id']))


    def get_thr_track_items(self):
        # the scope function info is stored in tokens list itself, as current scope stack would 
        # just contain 'top' at the end of ast traversal and when this function is called
        return self.thr_track_tokens
    

    def clear_thr_track_tokens(self):
        self.thr_track_tokens.clear()
            

    def track_cb_link_token(self, astnode):
        # store the function scope of token here, as current scope stack changes while traversing ast
        self.cb_link_track_tokens.append((self.get_function_scope_for_token(astnode), astnode.val['id']))

    
    def get_cb_link_track_items(self):
        # the scope function info is stored in tokens list itself, as current scope stack would 
        # just contain 'top' at the end of ast traversal and when this function is called
        return self.cb_link_track_tokens
    

    def clear_cb_link_track_tokens(self):
        self.cb_link_track_tokens.clear()


    def track_local_token(self, astnode):
        self.local_track_tokens.append((self.get_function_scope_for_token(astnode), astnode.val['id']))


    def get_local_track_items(self):
        # the scope function info is stored in tokens list itself, as current scope stack would 
        # just contain 'top' at the end of ast traversal and when this function is called
        return self.local_track_tokens
    

    def clear_local_track_tokens(self):
        self.local_track_tokens.clear()


    def track_spin_locations(self, astnode):
        self.spin_locations.append(self.get_function_scope_for_token(astnode))


    def get_spin_locations(self):
        return self.spin_locations


    def clear_spin_locations(self):
        self.spin_locations.clear()


    def track_init_func_scope(self):
        current_scope = self.get_current_scope()
        if current_scope and current_scope[0] == 'function':
            # track func scope names that create objects of classes
            self.init_func_scopes.append(current_scope[1])


    def get_init_func_scopes(self):
        return self.init_func_scopes


    def clear_init_func_scopes(self):
        self.init_func_scopes.clear()
