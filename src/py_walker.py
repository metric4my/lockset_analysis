#Copyright 2025 Purdue University.
#All rights reserved.


from graph_helper import GraphHelper
import py_walker_utils as pwu
import ast
import traceback


class PYWalker(ast.NodeVisitor):

    def __init__(self, gh, psh) -> None:
        self.gh = gh #GraphHelper()
        self.psh = psh #pwu.PYSymbolHelper()

        self.global_exec_id = None # for rclpy

        self.current_scope = [('module', 'top')] # stack of (scope type, scope name) from symtable package
        self.current_class_scope_name = None
        self.node_classes = []
        self.other_classes = []

        self.thr_track_tokens = [] # ast nodes tracking the potential processing in a new thread; e.g. spin
        self.cb_link_track_tokens= [] # tokens tracking the potential cb-cb invoke links; e.g. publish, service call, async calls with future
        self.local_track_tokens = [] # tokens tracking the local declrations of node, subscriber, timer, client, server, etc.
        self.spin_locations = [] # functions that call spin
        self.init_func_scopes = [] # function scope names that create objects (invoke __init__)

        self.disable_local_check = False


    ##TODO get_exec_val_id and get_node_val_id to account for variable access before its definition is encountered?
    ##TODO order of functions: process main that is top scope first?
    ##CHECK FOR BOTH PY AND CPP:
    ##TODO create future callback at call_async and add the callback link at the wait on future? 
    ##TODO use callback_fn type for callbacks and future type for setting result of the future object
    ##TODO find which all APIs can be called as a thread function, and then rearrange code to handle both thread and non-thread API call to avoid duplication


    def collect(self, astnode):
        self.setup_main_thread()
        # VISIT AST TREE IN POST ORDER: LEFT-RIGHT-NODE
        self.visit(astnode)


    def visit_Call(self, astnode: ast.Call):
        super().generic_visit(astnode)

        func_name = pwu.get_name(astnode.func)
        (func_obj, func_str) = pwu.get_object_and_name(astnode.func)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # PYTHON
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        # ''' threading.Thread(group=None, target=None, name=None, args=(), kwargs={}, *, daemon=None) '''
        # ''' Thread(group=None, target=None, name=None, args=(), kwargs={}, *, daemon=None) '''
        if func_name == 'threading.Thread' or (func_str == 'Thread' and not func_obj):
            func_arg = pwu.get_arg_at_pose_from_left(astnode, 'target', 2)
            thr_func = pwu.get_name(func_arg)
            (thr_func_obj, thr_func_str) = pwu.get_object_and_name(func_arg)

            if thr_func:
                thr_id = self.gh.setup_thread(1)
                pwu.set_val(astnode, 'id', thr_id)

                if thr_func_str == 'spin'  and thr_func_obj:
                    exec_id = None
                    node_id = None

                    if thr_func.startswith('rclpy.spin'):
                        thr_args = pwu.get_arg_at_pose_from_left(astnode, 'args', 4)
                        thr_kwargs = pwu.get_arg_at_pose_from_left(astnode, 'kwargs', 5)
                        node_arg = None
                        exec_arg = None
                        if thr_kwargs: # ast.Dict
                            node_arg = pwu.get_dict_value_for_key(thr_kwargs, 'node')
                            exec_arg = pwu.get_dict_value_for_key(thr_kwargs, 'executor')
                        if thr_args: # ast.Tuple
                            if not node_arg:
                                node_arg = pwu.get_tuple_value_at_pose(thr_args, 1)
                            if not exec_arg:
                                exec_arg = pwu.get_tuple_value_at_pose(thr_args, 2)

                        if node_arg:
                            node_id = self.get_node_val_id(node_arg)
                            if node_id:
                                exec_id = self.get_val_id(exec_arg) if exec_arg else self.get_global_exec_id()
                                if exec_id:
                                    self.gh.update_node_to_implicit_executor(node_id, exec_id)
                                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
                    else:
                        exec_token = thr_func_obj
                        exec_id = self.get_val_id(exec_token)

                    if exec_id:
                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE, 'node': node_id})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        pwu.set_val(astnode.func, 'id', (spin_id, exec_id))
                        self.gh.assign_thread_id((spin_id, exec_id), thr_id)

                        self.track_spin_locations(astnode.func)

                else:
                    self.gh.track_thread_function((None, thr_func), thr_id)
                

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        # ROS 2
        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
                    
        # ''' rclpy.executors.SingleThreadedExecutor(*,context=None) '''
        elif func_str == 'SingleThreadedExecutor':
            exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
            pwu.set_val(astnode, 'id', exec_id)
                        

        # ''' rclpy.executors.MultiThreadedExecutor(num_threads=None, *, context=None) '''
        elif func_str == 'MultiThreadedExecutor':
            thr_cnt = None

            thr_arg = pwu.get_arg_at_pose_from_left(astnode, 'num_threads', 1)
            if thr_arg:
                thr_cnt = pwu.get_numeric_value(thr_arg)

            if thr_cnt is None:
                thr_cnt = self.gh.ThrCount.CORES

            thr_id = self.gh.setup_thread(thr_cnt)

            exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': thr_cnt, 'thr_id': thr_id})
            pwu.set_val(astnode, 'id', exec_id)


        # ''' executor.add_node(node) '''
        elif func_str == 'add_node' and func_obj:
            exec_token = func_obj
            exec_id = self.get_val_id(exec_token)
            if exec_id:
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)
                if node_id:
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
                    

        ##TODO support executor.remove_node(node)?


        elif func_name == 'rclpy.get_global_executor':
            exec_id = self.get_global_exec_id()
            pwu.set_val(astnode, 'id', exec_id)


        # ''' rclpy.callback_groups.MutuallyExclusiveCallbackGroup() '''
        elif func_str == 'MutuallyExclusiveCallbackGroup':
            cbg_id = self.gh.add_graph_node('cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.MUTUALLY_EXCLUSIVE, 'add': True})
            pwu.set_val(astnode, 'id', cbg_id)
    

        # ''' rclpy.callback_groups.ReentrantCallbackGroup() '''
        elif func_str == 'ReentrantCallbackGroup':
            cbg_id = self.gh.add_graph_node('cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.REENTRANT, 'add': True})
            pwu.set_val(astnode, 'id', cbg_id)


        # NOTE: Add callback group to node in the subsequence functions, as there is no explicit add callback group function in rclpy unlike rclcpp.

        # ''' create_subscription(msg_type, topic, callback, qos_profile, *, callback_group=None, event_callbacks=None, raw=False)'''
        # FUTURE: support user defined callbacks for middleware events (event_callbacks)
        elif func_str == 'create_subscription' and func_obj:
            node_token = func_obj
            node_id = self.get_node_val_id(node_token)
            if node_id:
                cb_id = None
                cbg_id = None

                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback', 3)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SUB})
                    self.gh.track_callback_function(cb_func, cb_id)

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 5)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    pwu.set_val(astnode, 'id', cb_id)


        # ''' create_timer(timer_period_sec, callback, callback_group=None, clock=None) '''
        elif func_str == 'create_timer' and func_obj:
            node_token = func_obj 
            node_id = self.get_node_val_id(node_token)
            if node_id:
                cb_id = None
                cbg_id = None

                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback', 2)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    t_arg = pwu.get_arg_at_pose_from_left(astnode, 'timer_period_sec', 1)
                    t = pwu.get_time_value(t_arg)
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.TIMER, 'time': t})
                    self.gh.track_callback_function(cb_func, cb_id)

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 3)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    pwu.set_val(astnode, 'id', cb_id)


        # ''' create_rate(frequency, clock=None) '''
        elif func_str == 'create_rate' and func_obj:
            node_token = func_obj
            node_id = self.get_node_val_id(node_token)
            if node_id:
                cbg_id = self.gh.get_rate_cbgroup_of_node(node_id)

                t_arg = pwu.get_arg_at_pose_from_left(astnode, 'frequency', 1)
                t = pwu.get_time_value(t_arg)

                # convert frequency to time period
                if t != 0:
                    t = 1/t

                cb_id = self.gh.add_graph_node('rate_timer_cb', self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.TIMER, 'implicit': True, 'time': t})

                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                pwu.set_val(astnode, 'id', cb_id)
                self.disable_local_check = True
                    

        # ''' rate.sleep() '''
        # waits for implicit timer callback function to execute for coming out of sleep
        elif func_str == 'sleep' and func_obj:
            rate_token = func_obj
            cb_id = self.get_val_id(rate_token)
            if cb_id:
                pwu.set_val(astnode.func, 'id', cb_id)
                self.track_cb_link_token(astnode.func)


        # ''' create_service(srv_type, srv_name, callback, *, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None) '''
        elif func_str == 'create_service' and func_obj:
            node_token = func_obj
            node_id = self.get_node_val_id(node_token)
            if node_id:
                cb_id = None
                cbg_id = None

                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback', 3)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
                    self.gh.track_callback_function(cb_func, cb_id)

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 5)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    pwu.set_val(astnode, 'id', cb_id)


        # ''' create_client(srv_type, srv_name, *, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None) '''
        elif func_str == 'create_client' and func_obj:
            node_token = func_obj
            node_id = self.get_node_val_id(node_token)
            if node_id:
                cbg_id = None

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 4)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    # USE CBG WHILE CALLING SERVICE USING CLIENT
                    pwu.set_val(astnode, 'id', cbg_id)
                    self.disable_local_check = True


        # ''' client.call(request) '''
        # Internally calls call_async and add_done_callback, and waits for the done callback to finish
        elif func_str == 'call' and func_obj:      
            cli_token = func_obj
            cbg_id = self.get_val_id(cli_token)
            if cbg_id:
                cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                pwu.set_val(astnode.func, 'id', cb_id)
                self.track_cb_link_token(astnode.func)


        # ''' client.call_async(request)
        elif func_str == 'call_async' and func_obj:
            cli_token = func_obj
            cbg_id = self.get_val_id(cli_token)
            if cbg_id:
                # PROPOGATE CALLBACK GROUP TO FUTURE OBJECT
                pwu.set_val(astnode, 'id', cbg_id)
                self.disable_local_check = True


        # ''' future.add_done_callback(callback, *, context=None) '''
        elif func_str == 'add_done_callback' and func_obj:
            fu_token = func_obj
            cbg_id = self.get_val_id(fu_token)
            if cbg_id:
                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback', 1)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': False})

                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    self.gh.track_callback_function((None, cb_id), cb_id)

                    # propagate to future object and track cb link when waiting on future to get done
                    (fu_sym, sym_key) = self.get_var_symbol(fu_token)
                    if fu_sym: 
                        self.update_sym_val(fu_sym, 'd_cb_id', cb_id, sym_key)

                    # save cbg id in the callback function symbol to pass it on to the callback's future parameter
                    func_name = cb_func[1]
                    func_sym = self.get_function_symbol(func_name)
                    if func_sym:
                        unknown_cbg_id = self.get_sym_val(func_sym, 'id')
                        if unknown_cbg_id: 
                            # unknown_cbg id is set while processing the callback function code; change it to the client's cbg id
                            self.gh.relabel_graph_node(unknown_cbg_id, cbg_id)
                            
                        # store the client's cbg id in the callback to propogate it to callback's future parameter
                        self.update_sym_val(func_sym, 'id', cbg_id)


        # ''' action server '''
                        
        # ''' rclpy.action.server.ActionServer(node, action_type, action_name, execute_callback, *, callback_group=None, 
        #     goal_callback=<function default_goal_callback>, handle_accepted_callback=<function default_handle_accepted_callback>, 
        #     cancel_callback=<function default_cancel_callback>, goal_service_qos_profile=default, result_service_qos_profile=default, 
        #     cancel_service_qos_profile=default, feedback_pub_qos_profile=default, status_pub_qos_profile=default, result_timeout=900 sec) '''
        elif func_str == 'ActionServer':
            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt >= 4:
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)

                cbg_id = None

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 5)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    pwu.set_val(astnode, 'id', cbg_id)
                    self.disable_local_check = True
                    ##TODO store cbg_id as 'cbg', and callbacks as 'id'?

                    execute_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'execute_callback', 4)
                    cb_func_name = pwu.get_callback_function_name(execute_cb_arg, self.current_class_scope_name)
                    cb_func = self.psh.get_callback_function(cb_func_name)
                    if cb_func:
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'EXECUTE'})
                        self.gh.track_callback_function(cb_func, cb_id)
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                    cb_func = None
                    bsuff = False
                    goal_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'goal_callback', 6)
                    if goal_cb_arg:
                        cb_func_name = pwu.get_callback_function_name(goal_cb_arg, self.current_class_scope_name)
                        cb_func = self.psh.get_callback_function(cb_func_name)
                    else: #default
                        cb_func = (None, 'default_goal_callback')
                        bsuff = True
                    if cb_func:
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'GOAL'}, add_suffix=bsuff)
                        self.gh.track_callback_function(cb_func, cb_id)
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                    cb_func = None
                    bsuff = False
                    accepted_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'handle_accepted_callback', 7)
                    if accepted_cb_arg:
                        cb_func_name = pwu.get_callback_function_name(accepted_cb_arg, self.current_class_scope_name)
                        cb_func = self.psh.get_callback_function(cb_func_name)
                    else: #default
                        cb_func = (None, 'default_handle_accepted_callback')
                        bsuff = True
                    if cb_func:
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'ACCEPTED'}, add_suffix=bsuff)
                        self.gh.track_callback_function(cb_func, cb_id)
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                    cb_func = None
                    bsuff = False
                    cancel_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'cancel_callback', 6)
                    if cancel_cb_arg:
                        cb_func_name = pwu.get_callback_function_name(cancel_cb_arg, self.current_class_scope_name)
                        cb_func = self.psh.get_callback_function(cb_func_name)
                    else: #default
                        cb_func = (None, 'default_cancel_callback')
                        bsuff = True
                    if cb_func:
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'CANCEL'}, add_suffix=bsuff)
                        self.gh.track_callback_function(cb_func, cb_id)
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                        

        # ''' act_server.register_goal_callback(goal_callback) '''
        # if None, then unregisters any previously registered callback and sets it to default
        elif func_str == 'register_goal_callback' and func_obj:
            srv_token = func_obj
            cbg_id = self.get_val_id(srv_token)

            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt == 1:
                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'goal_callback', 1)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'GOAL'})
                    self.gh.track_callback_function(cb_func, cb_id)

                    if cbg_id and cb_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                        #pwu.set_val(astnode, 'id', cb_id)


        # ''' act_server.register_handle_accepted_callback(handle_accepted_callback) '''
        ##TODO replace any previously registered callback
        ##TODO if None, then unregisters any previously registered callback and sets it to default
        # this callback is called by the goal request processing function if the goal is accepted; 
        # default callback calls goal_handle.execute; if default is replaced with user provided callback, it should call goal_handle.execute
        elif func_str == 'register_handle_accepted_callback' and func_obj:
            srv_token = func_obj
            cbg_id = self.get_val_id(srv_token)

            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt == 1:
                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'handle_accepted_callback', 1)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'ACCEPTED'})
                    self.gh.track_callback_function(cb_func, cb_id)

                    if cbg_id and cb_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                        #pwu.set_val(astnode, 'id', cb_id)
                        

        # ''' act_server.register_cancel_callback(cancel_callback) '''
        ##TODO replace any previously registered callback
        ##TODO if None, then unregisters any previously registered callback and sets it to default
        elif func_str == 'register_cancel_callback' and func_obj:
            srv_token = func_obj
            cbg_id = self.get_val_id(srv_token)

            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt == 1:
                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'cancel_callback', 1)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'CANCEL'})
                    self.gh.track_callback_function(cb_func, cb_id)

                    if cbg_id and cb_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                        #pwu.set_val(astnode, 'id', cb_id)
             

        # ''' act_server.register_execute_callback(execute_callback)
        ##TODO Must not be None
        # user provided execute callback should call goal_handle.publish_feedback if desired
        elif func_str == 'register_execute_callback' and func_obj:
            srv_token = func_obj
            cbg_id = self.get_val_id(srv_token)

            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt == 1:
                cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'execute_callback', 1)
                cb_func_name = pwu.get_callback_function_name(cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV, 'act_type': 'EXECUTE'})
                    self.gh.track_callback_function(cb_func, cb_id)

                    if cbg_id and cb_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                        #pwu.set_val(astnode, 'id', cb_id)
                        

        # ''' action client '''
        # ''' rclpy.action.client.ActionClient(node, action_type, action_name, *, callback_group=None, 
        #     goal_service_qos_profile=default, result_service_qos_profile=default, cancel_service_qos_profile=default, 
        #     feedback_sub_qos_profile=default, status_sub_qos_profile=default) '''
        elif func_str == 'ActionClient':
            arg_cnt = pwu.get_arg_count(astnode)
            if arg_cnt >= 3:
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)
                
                cbg_id = None

                cbg_arg = pwu.get_arg_at_pose_from_left(astnode, 'callback_group', 4)
                if cbg_arg:
                    cbg_id = self.get_val_id(cbg_arg)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    pwu.set_val(astnode, 'id', cbg_id)
                    self.disable_local_check = True


        # ''' action_client.send_goal(goal, **kwargs) '''
        # Internally calls send_goal_async and add_done_callback, and waits for the done callback to finish
        elif func_str == 'send_goal' and func_obj:      
            cli_token = func_obj
            cbg_id = self.get_val_id(cli_token)
            if cbg_id:
                cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                pwu.set_val(astnode.func, 'id', cb_id)
                self.track_cb_link_token(astnode.func)

                feed_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'feedback_callback', 2)
                cb_func_name = pwu.get_callback_function_name(feed_cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                    self.gh.track_callback_function(cb_func, cb_id)
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    

        # ''' action_client.send_goal_async(goal, feedback_callback=None, goal_uuid=None) '''
        elif func_str == 'send_goal_async' and func_obj:
            handle_token = func_obj
            cbg_id = self.get_val_id(handle_token)
            if cbg_id:
                # PROPOGATE CALLBACK GROUP TO FUTURE OBJECT
                pwu.set_val(astnode, 'id', cbg_id)
                self.disable_local_check = True

                feed_cb_arg = pwu.get_arg_at_pose_from_left(astnode, 'feedback_callback', 2)
                cb_func_name = pwu.get_callback_function_name(feed_cb_arg, self.current_class_scope_name)
                cb_func = self.psh.get_callback_function(cb_func_name)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                    self.gh.track_callback_function(cb_func, cb_id)
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


        # ''' future.result() '''
        elif func_str == 'result' and func_obj:
            future_token = func_obj
            cbg_id = self.get_val_id(future_token)
            if not cbg_id:
                (future_sym, sym_key) = self.get_var_symbol(future_token)
                if future_sym and self.psh.is_parameter_symbol(future_sym):
                    current_scope = self.get_current_scope()
                    if current_scope[0] == 'function':
                        func_name = current_scope[1]
                        func_sym = self.get_function_symbol(func_name)
                        if func_sym:
                            cbg_id = self.get_sym_val(func_sym, 'id')
                            if not cbg_id:
                                cbg_id = self.gh.add_graph_node('unknown_cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.UNKNOWN, 'add': True})
                                self.update_sym_val(func_sym, 'id', cbg_id)

            if cbg_id:
                 # PROPOGATE CALLBACK GROUP TO RESULT OBJECT, e.g., action client goal handle
                pwu.set_val(astnode, 'id', cbg_id)
                self.disable_local_check = True
                    

        # ''' action_client_goal_handle.cancel_goal() '''
        # Internally calls cancel_goal_async and add_done_callback, and waits for the done callback to finish
        elif func_str == 'cancel_goal' and func_obj:      
            handle_token = func_obj
            cbg_id = self.get_val_id(handle_token)
            if cbg_id:
                cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                pwu.set_val(astnode.func, 'id', cb_id)
                self.track_cb_link_token(astnode.func)
                    

        # ''' action_client_goal_handle.cancel_goal_async() '''
        elif func_str == 'cancel_goal_async' and func_obj:
            handle_token = func_obj
            cbg_id = self.get_val_id(handle_token)
            if cbg_id:
                # PROPOGATE CALLBACK GROUP TO FUTURE OBJECT
                pwu.set_val(astnode, 'id', cbg_id)
                self.disable_local_check = True
                    

        # ''' action_client_goal_handle.get_result() '''
        # Internally calls get_result_async and add_done_callback, and waits for the done callback to finish
        elif func_str == 'get_result' and func_obj:      
            handle_token = func_obj
            cbg_id = self.get_val_id(handle_token)
            if cbg_id:
                cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                pwu.set_val(astnode.func, 'id', cb_id)
                self.track_cb_link_token(astnode.func)
                    

        # ''' action_client_goal_handle.get_result_async() '''
        elif func_str == 'get_result_async' and func_obj:
            handle_token = func_obj
            cbg_id = self.get_val_id(handle_token)
            if cbg_id:
                # PROPOGATE CALLBACK GROUP TO FUTURE OBJECT
                pwu.set_val(astnode, 'id', cbg_id)
                self.disable_local_check = True
            

        # ''' rclpy.spin(node, executor=None) '''
        # ''' executor.spin() '''
        elif func_str == 'spin' and func_obj:
            exec_id = None
            node_id = None

            if func_name == 'rclpy.spin':
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)
                if node_id:
                    exec_arg = pwu.get_arg_at_pose_from_left(astnode, 'executor', 2)
                    exec_id = self.get_val_id(exec_arg) if exec_arg else self.get_global_exec_id()
                    if exec_id:
                        self.gh.update_node_to_implicit_executor(node_id, exec_id)
                        self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)    
            else:
                exec_token = func_obj
                exec_id = self.get_val_id(exec_token)

            if exec_id:      
                ##TODO add control edges for spin to detect wrong location of spin invocation?
                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE, 'node': node_id})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode.func, 'id', (spin_id, exec_id))
                self.track_thr_token(astnode.func)

                self.track_spin_locations(astnode.func)


        # ''' rclpy.spin_once(node, *, executor=None, timeout_sec=None) '''
        # ''' executor.spin_once(timeout_sec=None) '''
        elif func_str == 'spin_once' and func_obj:
            exec_id = None
            node_id = None
            t_arg = None

            if func_name == 'rclpy.spin_once':
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)
                if node_id:
                    exec_arg = pwu.get_arg_at_pose_from_left(astnode, 'executor', 2)
                    exec_id = self.get_val_id(exec_arg) if exec_arg else self.get_global_exec_id()
                    if exec_id:
                        self.gh.update_node_to_implicit_executor(node_id, exec_id)
                        self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                        t_arg = pwu.get_arg_at_pose_from_left(astnode, 'timeout_sec', 3)
            else:
                exec_token = func_obj
                exec_id = self.get_val_id(exec_token)
                if exec_id:
                    t_arg = pwu.get_arg_at_pose_from_left(astnode, 'timeout_sec', 1)
                        
            if exec_id:
                t = pwu.get_time_value(t_arg) if t_arg else None
                if t is None:
                    t = -1  # block indefinitely
                          
                ##TODO add control edges for spin to detect wrong location of spin invocation?
                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.ONE, 'time': t, 'node': node_id})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode.func, 'id', (spin_id, exec_id))
                self.track_thr_token(astnode.func)

                self.track_spin_locations(astnode.func)

        
        # ''' rclpy.spin_until_future_complete(node, future, executor=None, timeout_sec=None) '''
        # ''' executor.spin_until_future_complete(future, timeout_sec=None) '''
        elif func_str == 'spin_until_future_complete' and func_obj:
            exec_id = None
            node_id = None
            t_arg = None
            fu_arg = None
        
            if func_name == 'rclpy.spin_until_future_complete':
                node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node', 1)
                node_id = self.get_node_val_id(node_arg)
                if node_id:
                    exec_arg = pwu.get_arg_at_pose_from_left(astnode, 'executor', 3)
                    exec_id = self.get_val_id(exec_arg) if exec_arg else self.get_global_exec_id()
                    if exec_id:
                        self.gh.update_node_to_implicit_executor(node_id, exec_id)
                        self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                        t_arg = pwu.get_arg_at_pose_from_left(astnode, 'timeout_sec', 4)
                        fu_arg = pwu.get_arg_at_pose_from_left(astnode, 'future', 2)
            else:
                exec_token = func_obj
                exec_id = self.get_val_id(exec_token)
                if exec_id:
                    t_arg = pwu.get_arg_at_pose_from_left(astnode, 'timeout_sec', 2)
                    fu_arg = pwu.get_arg_at_pose_from_left(astnode, 'future', 1)

            if exec_id:
                t = pwu.get_time_value(t_arg) if t_arg else None
                if t is None:
                    t = -1  # block indefinitely

                ##TODO add control edges for spin to detect wrong location of spin invocation?
                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.FUTURE, 'time': t, 'node': node_id})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                pwu.set_val(astnode.func, 'id', (spin_id, exec_id))
                self.track_thr_token(astnode.func)

                self.track_spin_locations(astnode.func)

                # ADD IMPLICIT FUTURE DONE CALLBACK TO REPRESENT WAIT UNTIL FUTURE IS SET
                cbg_id = self.get_val_id(fu_arg)
                if cbg_id:
                    # represent 'setting future with service result task that is scheduled based on callback group' as implicit done callback
                    cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    self.gh.track_callback_function((None, cb_id), cb_id)

                    pwu.set_val(astnode.func, 'id', cb_id)
                    self.track_cb_link_token(astnode.func)


        # ''' rclpy.create_node(node_name, ...) '''
        elif func_name == 'rclpy.create_node':
            node_arg = pwu.get_arg_at_pose_from_left(astnode, 'node_name', 1)
            node_id = pwu.get_string_value(node_arg)
            if node_id:
                self.gh.update_node(node_id, is_active=True)
                pwu.set_val(astnode, 'id', node_id)
                pwu.set_val(astnode, 'is_active', True)
                
                pwu.set_val(astnode.func, 'id', (node_id,))
                self.track_thr_token(astnode.func)


        # node class constructor call
        ##TODO node_classes will not have an entry, when classdef is not visited yet. How to handle this case?
        elif func_name in self.node_classes or func_name in self.other_classes:
            # track function scopes that create objects i.e. invoke __init__ method of classes.
            self.track_init_func_scope()

            if func_name in self.node_classes:
                node_id = func_name
                self.gh.update_node(node_id, is_active=True, setup=False)
                pwu.set_val(astnode, 'id', node_id)
                pwu.set_val(astnode, 'is_active', True)

                pwu.set_val(astnode.func, 'id', (node_id,))
                self.track_thr_token(astnode.func)


    def visit_Await(self, astnode: ast.Await):
        fu_token = astnode.value
        (fu_sym, sym_key) = self.get_var_symbol(fu_token)
        if fu_sym:
            cb_id = self.get_sym_val(fu_sym, 'd_cb_id', sym_key)
            if cb_id:
                pwu.set_val(astnode, 'id', cb_id)
                self.track_cb_link_token(astnode)
        

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
                self.disable_local_check = False


    def visit_AnnAssign(self, astnode: ast.AnnAssign):
        super().generic_visit(astnode)

        if hasattr(astnode.value, 'val'):
            if astnode.target:
                t = astnode.target
                (t_sym, sym_key) = self.get_var_symbol(t)

                if t_sym:
                    for r_key, r_val in astnode.value.val.items():
                        self.update_sym_val(t_sym, r_key, r_val, sym_key)

                        # track local declarations of objects like node, subscriber, timer, client, server, etc.
                        if (not self.disable_local_check) and r_key == 'id' and self.psh.is_local_symbol(t_sym):
                            #self.gh.mark_graph_node_as_local(r_val)
                            self.track_local_token(astnode.value)

            if self.disable_local_check:
                self.disable_local_check = False


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


    visit_AsyncFunctionDef = visit_FunctionDef


    def visit_ClassDef(self, astnode: ast.ClassDef):
        class_name = astnode.name

        is_node_class = False
        for base_class in astnode.bases:
            if isinstance(base_class, ast.Name):
                if base_class.id == 'Node' or base_class.id == 'rclpy.Node':
                    self.gh.update_node(class_name)
                    self.node_classes.append(class_name)
                    is_node_class = True
                    ##TODO clear node_classes after processing python package files? 
        if not is_node_class:
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

        if isinstance(astnode.test, ast.Call):
            func_name = pwu.get_name(astnode.test.func)
            if func_name == 'rclpy.ok':
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
    

    def get_function_symbol(self, func_name):
        if '::' in func_name:
            func_name = func_name.split('::')[1] #remove the class name
        if func_name.startswith('__') and func_name != '__init__':
            func_name = '_' + self.current_class_scope_name + func_name
        nf_scope = self.get_current_non_function_scope()
        func_sym = self.psh.get_symbol(func_name, nf_scope)
        return func_sym
    

    def get_sym_val(self, sym, val_key, sym_key=''):
        if sym_key:
            val_key = sym_key + '_' + val_key

        return self.psh.get_val(sym, val_key)


    def update_sym_val(self, sym, new_key, new_val, sym_key=''):
        if sym_key:
            new_key = sym_key + '_' + new_key

        self.psh.set_val(sym, new_key, new_val)
    

    def get_val_id(self, astnode):
        val_id = None

        (var_sym, sym_key) = self.get_var_symbol(astnode)
        if var_sym:
            val_id = self.get_sym_val(var_sym, 'id', sym_key)
        else:
            val_id = pwu.get_val(astnode, 'id')

        return val_id
    

    def get_node_val_id(self, astnode):
        if pwu.get_name(astnode) == 'self':
            if self.current_class_scope_name in self.node_classes:
                return self.current_class_scope_name
            else:
                return None
        else:
            return self.get_val_id(astnode)
        

    def get_global_exec_id(self):
        if not self.global_exec_id:
            # INITIALIZE GLOBAL EXECUTOR
            self.global_exec_id = self.gh.add_graph_node('global_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
        
        return self.global_exec_id
    

    def get_current_scope(self):
        if len(self.current_scope) >= 1:
            return self.current_scope[-1]
    

    def get_main_scope(self):
        if len(self.current_scope) >= 1:
            return self.current_scope[0]
        

    def get_current_non_function_scope(self):
        nf_scope = None
        for scope in reversed(self.current_scope):
            if scope[0] != 'function':
                nf_scope = scope
                break
        return nf_scope


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
        # return self.local_track_tokens
        
        return [] #disable local check for rclpy, as it stores references for the objects.
    

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


if __name__ == '__main__':
    code = '''
x = 5
y = x
    '''

    psh = pwu.PYSymbolHelper()
    psh.generate_symbol_table(code)
    pw = PYWalker(None, psh)
    
    tree = ast.parse(code)
    pw.collect(tree)
