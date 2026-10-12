#Copyright 2025 Purdue University.
#All rights reserved.


import cpp_walker_utils as cwu
from graph_helper import GraphHelper


class CPPWalker:
    
    def __init__(self, gh) -> None:
        self.gh = gh #GraphHelper()

        self.was_some_value_changed = False
        self.found_values_in_this_tree = False
        self.disable_local_check = False

        self.thr_track_tokens = [] # tokens tracking the potential processing in a new thread; e.g. spin
        self.cb_link_track_tokens= [] # tokens tracking the potential cb-cb invoke links; e.g. publish, service call, async calls with future
        self.local_track_tokens = [] # tokens tracking the local declrations of node, subscriber, timer, client, server, etc.
        self.spin_locations = [] # functions that call spin


    def collect(self, root_token):
        cwu.generic_recurse_and_apply_function(root_token, self.process_self_pointer)
        cwu.generic_recurse_and_apply_function(root_token, self.process_make_shared_or_unique)
        cwu.generic_recurse_and_apply_function(root_token, self.process_thread_construction)
    
        cwu.generic_recurse_and_apply_function(root_token, self.process_node_construction)
        cwu.generic_recurse_and_apply_function(root_token, self.process_node_base_interface)
        cwu.generic_recurse_and_apply_function(root_token, self.process_callback_group_type)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_callback_group)
        cwu.generic_recurse_and_apply_function(root_token, self.process_executor_construction)

        cwu.generic_recurse_and_apply_function(root_token, self.process_create_subscription)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_wall_timer)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_service)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_client)
        cwu.generic_recurse_and_apply_function(root_token, self.process_async_send_request)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_action_server)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_action_client)
        cwu.generic_recurse_and_apply_function(root_token, self.process_sendgoaloptions)
        cwu.generic_recurse_and_apply_function(root_token, self.process_async_send_goal)
        cwu.generic_recurse_and_apply_function(root_token, self.process_async_get_result)
        cwu.generic_recurse_and_apply_function(root_token, self.process_async_cancel_goal)
        cwu.generic_recurse_and_apply_function(root_token, self.process_add_on_set_parameters_callback)
        cwu.generic_recurse_and_apply_function(root_token, self.process_message_filter_subscriber_synchronizer)

        cwu.generic_recurse_and_apply_function(root_token, self.process_future_wait_get)
        cwu.generic_recurse_and_apply_function(root_token, self.process_move)
        cwu.generic_recurse_and_apply_function(root_token, self.process_copy_construct)
        cwu.generic_recurse_and_apply_function(root_token, self.process_assignment)
        cwu.generic_recurse_and_apply_function(root_token, self.process_while)
        
        cwu.generic_recurse_and_apply_function(root_token, self.process_add_node_to_executor)
        cwu.generic_recurse_and_apply_function(root_token, self.process_add_callback_group_to_executor)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin_some)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin_all)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin_once)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin_until_future_complete)


    def collect_from_function(self, function_object):
        self.process_lifecycle_callbacks(function_object)
        self.process_main(function_object)


    ####TODO uncomment
    #def reset_current_node_id(self):
    #    pass


    ##TODO post process to add callback groups of node to the executors stored in the map, to support implicit add node to executor in the spin call
    ##TODO do we need to have get_cbg_val_id similar to executor? Process ros node constructor subtree first before any other function subtree?
    ##TODO at the declaration of executor check if there is thr_count is stored in variable's val and update that val if necessary, also update the attribute of the executor graph node
    ##TODO get_node_val_id similar to get_exec_val_id to account for node variable usage before encountering its definition and to relable node when definition?

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # CPP
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -   

    ''' = '''
    def process_assignment(self, token, left_token, right_token):
        if token.str == '=' and token.isAssignmentOp and right_token.val:
            (left_var, left_str) = cwu.get_var_token_and_var_name(left_token)
            if left_var and left_var.variable:
                ##TODO check the left var type? ##
                left_var.variable.val = right_token.val

                # track local declarations of objects like node, subscriber, timer, client, server, etc.
                if (not self.disable_local_check) and 'id' in right_token.val and cwu.is_local_symbol(left_var.variable):
                    #self.gh.mark_graph_node_as_local(right_token.val['id'])
                    self.track_local_token(right_token)
                
            if self.disable_local_check:
                self.disable_local_check = False


    ''' obj1(obj2) 
        obj1(std::move(obj2))
        where T(obj1)==T(obj2) '''
    def process_copy_construct(self, token, left_token, right_token):
        if token.str == '(' and left_token and right_token:
            (left_var, left_str) = cwu.get_var_token_and_var_name(left_token)
            if left_var and left_var.variable:
                (right_var, right_str) = cwu.get_var_token_and_var_name(right_token)
                if right_var and right_var.variable:
                    if cwu.find_sanitized_variable_type(left_var.variable) == cwu.find_sanitized_variable_type(right_var.variable):
                        left_var.variable.val = right_var.variable.val
                elif cwu.is_match(right_token, '(') and cwu.is_match_next(token, ['std', '::', 'move']):
                    left_var.variable.val = right_token.val


    ''' shared_from_this()
        this 
        '''
    def process_self_pointer(self, token, left_token, right_token):
        if token.str == 'shared_from_this' and cwu.is_match_next(token, ['(', ')']):
            paren_token = token.next
            node_id = cwu.get_class_scope_name_for_token(token)
            if node_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', node_id)
            
                if cwu.is_match(paren_token.astParent, '='):
                    self.disable_local_check = True

        elif token.str == 'this':
            node_id = cwu.get_class_scope_name_for_token(token)
            if node_id:
                token.val = cwu.update_val_dict(token.val, 'id', node_id)

                if cwu.is_match(token.astParent, '='):
                    self.disable_local_check = True


    ''' std::make_shared<[NODE-TYPE]>(...) 
        std::make_shared<rclcpp::executors::EXECUTOR-TYPE]>(...)
        std::make_unique<std::thread>(...)
        '''
    ##TODO process make_shared and make_unique for all the other objects' functions and process only node part here by checking the type not in objects'
    ## done the above for some objects like executor, thread and synchronizer
    def process_make_shared_or_unique(self, token,left_token, right_token):
        if token.str in ['make_shared', 'make_unique']:    
            paren_token = None

            if cwu.is_match(token.astParent, '::') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent

            if paren_token and cwu.is_match(token.next, '<') and cwu.is_match(token.next.link, '>'):
                type_token = token.next.link.previous
                if type_token:
                    if type_token.str == '>' and cwu.is_match(type_token.link, '<') and type_token.link.previous:
                        type_token = type_token.link.previous

                    if type_token.str.endswith('Executor') or type_token.str == 'thread' or type_token.str.endswith('Synchronizer') or \
                            type_token.str == 'Node' or type_token.str.endswith("Client"):
                        token.val = cwu.update_val_dict(token.val, 'type_token', type_token)
                        token.val = cwu.update_val_dict(token.val, 'paren_token', paren_token)
                    else:
                        ##TODO check for node_token.type-scope.type == 'Class'? - do check, but currently disabled
                        node_id = self.construct_node(paren_token, type_token)
                        if node_id:
                            type_token.val = cwu.update_val_dict(type_token.val, 'id', (node_id,))
                            self.track_thr_token(type_token)
                                        

    ''' std::move(obj)
        '''
    def process_move(self, token, left_token, right_token):
        if token.str == 'move' and cwu.is_match_previous(token, ['std', '::']) and cwu.is_match(token.previous.astParent, '('):
            paren_token = token.previous.astParent
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            if arg_cnt == 1:
                obj_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (obj_var, obj_str) = cwu.get_var_token_and_var_name(obj_token)
                if obj_var and obj_var.variable:
                    paren_token.val = obj_var.variable.val


    ''' boost/std::thread th(func, args) => args by value or ref
        boost/std::thread th(&func, &obj) => obj.func
        std::make_unique<std::thread>(...)
        '''
    def process_thread_construction(self, token, left_token, right_token):
        paren_token = None
        thr_token = None

        if token.str == 'thread':
            if cwu.is_match(token.previous, '::') and cwu.is_match(token.previous.astParent, '('):
                paren_token = token.previous.astParent
                thr_token = token.previous.astOperand2
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                thr_token = token.next
            elif cwu.is_match(token.astParent, '.'):
                print("Found . leading thread, temporarily skipping")
                return
        elif token.str in ['make_shared', 'make_unique'] and token.val:
            type_token = token.val.get('type_token')
            if type_token and type_token.str == 'thread':
                paren_token = token.val.get('paren_token')

        if paren_token:        
            thr_id = self.construct_thread(paren_token)
            
            if thr_id:
                if thr_token:
                    (thr_var, thr_str) = cwu.get_var_token_and_var_name(thr_token)
                    if thr_var and thr_var.variable:
                        thr_var.variable.val = cwu.update_val_dict(thr_var.variable.val, 'id', thr_id)
                    
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', thr_id)


    ''' std::future.wait()
        std::future.wait_for(timeout)
        std::future.wait_until(time)
        std::future.get()
        note: all the above functions are blocking.
        '''
    def process_future_wait_get(self, token, left_token, right_token):
        if (token.str.startswith('wait') or token.str == 'get') and cwu.is_match(token.astParent, '.') and \
                cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            fu_token = token.astParent.astOperand1
            (fu_var, fu_str) = cwu.get_var_token_and_var_name(fu_token)
            # the type of future variable can be auto
            if fu_var and fu_var.variable:
                fu_type = cwu.find_sanitized_variable_type(fu_var.variable)
                if not (fu_type.endswith('future') or fu_type.endswith('auto')):
                    return
                cbg_id = cwu.get_val_id(fu_token)
                if cbg_id:
                    arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                    t = None
                    
                    if arg_cnt == 1:
                        t_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                        t = cwu.get_time_value(t_token)

                    if t:
                        cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True, 'time': t})
                    else:
                        cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True})

                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    self.gh.track_callback_function((None, cb_id), cb_id)

                    token.val = cwu.update_val_dict(token.val, 'id', cb_id)
                    self.track_cb_link_token(token)


    ##TODO propagate graph node ids across function calls (return, function call)??

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # ROS 2
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    ##TODO node construction via object declaration e.g. PlannerServer ps
    ''' rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("my_node");
        rclcpp::Node node("my_node");
        node = std::make_shared<rclcpp::Node>("my_node")
        '''
    def process_node_construction(self, token, left_token, right_token):
        paren_token = None
        node_token = None

        if token.str == 'Node' and cwu.is_match_previous(token, ['rclcpp', '::']):
            if cwu.is_match_next(token, ['::', 'make_shared', '(']):
                paren_token = token.next.next.next
            elif cwu.is_match(token.next.next, '('):
                paren_token = token.next.next
                node_token = token.next # variable
            elif cwu.is_match(token.next, '('):
                paren_token = token.next
        elif token.str in ['make_shared', 'make_unique'] and token.val:
            type_token = token.val.get('type_token')
            if type_token and type_token.str == 'Node':
                paren_token = token.val.get('paren_token')

        if paren_token:
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            id_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)

            node_id = self.construct_node(paren_token, id_token)
            if node_id:
                id_token.val = cwu.update_val_dict(id_token.val, 'id', (node_id,))
                self.track_thr_token(id_token)

                if node_token:
                    (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
                    if node_var and node_var.variable:
                        node_var.variable.val = cwu.update_val_dict(node_var.variable.val, 'is_active', True)
                        node_var.variable.val = cwu.update_val_dict(node_var.variable.val, 'id', node_id)


    ''' rclcpp::Node::get_node_base_interface()
        '''
    def process_node_base_interface(self, token, left_token, right_token):
        if token.str == 'get_node_base_interface':
            paren_token = None
            node_id = ''
            if cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                node_token = token.astParent.astOperand1
                node_id = cwu.get_val_id(node_token)
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                node_id = self.get_node_id_for_token(token)

            if node_id:
                self.gh.update_node(node_id)
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', node_id)
                self.disable_local_check = True


    ''' rclcpp::CallbackGroupType::MutuallyExclusive
        rclcpp::CallbackGroupType::Reentrant 
        '''
    def process_callback_group_type(self, token, left_token, right_token):
        val = None
        if token.str == 'MutuallyExclusive':
           val = cwu.update_val_dict(val, 'group_type', self.gh.CbGroup.MUTUALLY_EXCLUSIVE)
        elif token.str == 'Reentrant':
           val = cwu.update_val_dict(val, 'group_type', self.gh.CbGroup.REENTRANT)

        if val:
            if token.astParent.str == '::':
                token = token.astParent
            token.val = val


    ''' create_callback_group (rclcpp::CallbackGroupType group_type) FOXY
        create_callback_group (rclcpp::CallbackGroupType group_type, bool automatically_add_to_executor_with_node=true) GALACTIC 
        '''
    def process_create_callback_group(self, token, left_token, right_token):
        if token.str == 'create_callback_group' and cwu.is_match(token.next, '('):
            paren_token = token.next
            node_id = ''

            if cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                #NODE OBJECT
                node_id = cwu.get_class_scope_name_for_token(token)
            elif cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                # NODE OBJECT
                if token.astParent.astOperand1:
                    node_token = token.astParent.astOperand1
                    node_id = cwu.get_val_id(node_token)
        
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            # CALLBACK GROUP TYPE
            type_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)

            if type_arg.val:
                paren_token.val = type_arg.val
            else:
                (type_var, type_str) = cwu.get_var_token_and_var_name(type_arg)
                if (type_var and type_var.variable):
                    paren_token.val = type_var.variable.val

            # BY DEFAULT, ADD CALLBACK GROUP TO EXECUTOR AUTOMATICALLY
            add = True
            if arg_cnt == 2:
                add_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                add = False if (cwu.get_numeric_value(add_arg) == 0) else True

            paren_token.val = cwu.update_val_dict(paren_token.val, 'add', add)
    
            cbg_id = self.gh.add_graph_node("cbg", self.gh.NType.CALLBACK_GROUP, paren_token.val)
            paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)

            self.gh.update_node(node_id)
            self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)


    ''' std::make_shared<rclcpp::executors::EXECUTOR-TYPE]>(...)
        rclcpp::executors::MultiThreadedExecutor executor(options, 5, ...)
        note: constructor has all default arguments with thread count at the second position
        thread count: if not specified, it equals to the number of cores
        '''
    def process_executor_construction(self, token, left_token, right_token):
        paren_token = None
        exec_token = None
    
        if token.str.endswith('Executor'):
            if cwu.is_match(token.previous, '::') and cwu.is_match(token.previous.astParent, '('):
                paren_token = token.previous.astParent
                exec_token = token.previous.astOperand2
            elif cwu.is_match(token.astParent, '('):
                ##TODO verify if the astOperand1 of paren token is variable or 'MultiThreadedExecutor'
                paren_token = token.astParent
                exec_token = token.next
        elif token.str in ['make_shared', 'make_unique'] and token.val:
            type_token = token.val.get('type_token')
            if type_token and type_token.str.endswith('Executor'):
                paren_token = token.val.get('paren_token')
                token = type_token

        if paren_token:
            (exec_id, thr_cnt, thr_id) = self.construct_executor(paren_token, token)

            if exec_id:    
                if exec_token:        
                    (exec_var, exec_str) = cwu.get_var_token_and_var_name(exec_token)
                    if exec_var and exec_var.variable:
                        exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'id', exec_id)
                        exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'thr_count', thr_cnt)
                        exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'thr_id', thr_id)

                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', exec_id)
                paren_token.val = cwu.update_val_dict(paren_token.val, 'thr_count', thr_cnt)
                paren_token.val = cwu.update_val_dict(paren_token.val, 'thr_id', thr_id)


    ''' add_node(node, ...)
        '''
    def process_add_node_to_executor(self, token, left_token, right_token):
        if token.str == 'add_node' and cwu.is_match(token.astParent, '.'):
            exec_token = token.astParent.astOperand1
            exec_id = self.get_executor_val_id(exec_token)

            if cwu.is_match(token.astParent.astParent, '('):
                node_token = token.astParent.astParent.astOperand2
                node_id = cwu.get_val_id(node_token)

            if exec_id and node_id:
                self.gh.update_node(node_id)
                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
                cbg_list = self.gh.get_callback_groups_of_node(node_id)
                for cbg_id in cbg_list:
                    self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)


    ''' add_callback_group(group, ...)
        '''
    def process_add_callback_group_to_executor(self, token, left_token, right_token):
        if token.str == 'add_callback_group' and cwu.is_match(token.astParent, '.'):
            exec_token = token.astParent.astOperand1
            exec_id = self.get_executor_val_id(exec_token)

            if cwu.is_match(token.astParent.astParent, '('):
                cbg_token = token.astParent.astParent.astOperand2
                cbg_id = cwu.get_val_id(cbg_token)

            if exec_id and cbg_id:
                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)


    ##TODO support remove_node(node, ...); might need control edges to support it
    ##TODO support remove_callback_group(cbg, ...); might need control edges to support it 


    ''' The following functions with rclcpp:: prefix do add_node and remove_node: ##TODO support remove_node?
        rclcpp::spin(node)
        executor.spin() 
        '''
    def process_spin(self, token, left_token, right_token):
        if token.str == 'spin' and cwu.is_match(token.next, '('):
            if cwu.is_match_previous(token, ['rclcpp', '::']):
                if token.next.astOperand2:
                    node_token = token.next.astOperand2
                    node_id = cwu.get_val_id(node_token)
                    
                    if node_id:
                        # STORE NODE TO DEFAULT EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                        exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                        self.gh.update_node_to_implicit_executor(node_id, exec_id)
                        self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)          

                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                        self.track_thr_token(token)

                        self.track_spin_locations(token)

            elif cwu.is_match(token.astParent, '.') and token.astParent.astOperand1:
                exec_token = token.astParent.astOperand1
                exec_id = self.get_executor_val_id(exec_token)
                
                if exec_id:
                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)
                    
                    self.track_spin_locations(token)


    ''' The following functions with rclcpp:: prefix do add_node and remove_node: ##TODO support remove_node?
        rclcpp::spin_some(node)
        executor.spin_node_some(node)
        executor.spin_some(max_duration=0) # ns
        '''
    def process_spin_some(self, token, left_token, right_token):
        if token.str.startswith('spin') and token.str.endswith('some') and cwu.is_match(token.next, '('):
            t = 0 # no limit ##TODO change it to negative value for consistency across all spin APIs in rclcpp and rclpy to represent blocking?

            if cwu.is_match_previous(token, ['rclcpp', '::']):
                if token.next.astOperand2:
                    node_token = token.next.astOperand2
                    node_id = cwu.get_val_id(node_token)      
                    if node_id:
                        # STORE NODE TO DEFAULT EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                        exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                        self.gh.update_node_to_implicit_executor(node_id, exec_id)
                        self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE, 'time': t})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                        self.track_thr_token(token)

                        self.track_spin_locations(token)

            elif cwu.is_match(token.astParent, '.') and token.astParent.astOperand1:
                exec_token = token.astParent.astOperand1
                exec_id = self.get_executor_val_id(exec_token)                
                if exec_id:
                    if token.str == 'spin_node_some':
                        if token.next.astOperand2:
                            node_token = token.next.astOperand2
                            node_id = cwu.get_val_id(node_token)
                            if node_id:
                                # STORE NODE TO (DEFAULT) EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                                self.gh.update_node_to_implicit_executor(node_id, exec_id)
                                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                                ##TODO add control edges for spin to detect wrong location of spin invocation?
                                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE, 'time': t})
                                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                                token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                                self.track_thr_token(token)

                                self.track_spin_locations(token)
                    elif token.str == 'spin_some':
                        if token.next.astOperand2:
                            t_arg = token.next.astOperand2
                            t = cwu.get_time_value(t_arg)

                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE, 'time': t})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                        self.track_thr_token(token)

                        self.track_spin_locations(token)


    ''' The following functions with rclcpp:: prefix do add_node and remove_node: ##TODO support remove_node?
        rclcpp::spin_all(node, max_duration) # ns
        executor.spin_node_all(node, max_duration) # ns
        executor.spin_all(max_duration) # ns
        '''
    def process_spin_all(self, token, left_token, right_token):
        if token.str.startswith('spin') and token.str.endswith('all') and cwu.is_match(token.next, '('):
            paren_token = token.next
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
            t = cwu.get_time_value(t_arg)

            if cwu.is_match_previous(token, ['rclcpp', '::']):
                node_token = cwu.get_comma_separated_token_at_pose_from_right(arg_cnt)
                node_id = cwu.get_val_id(node_token)      
                if node_id:
                    # STORE NODE TO DEFAULT EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                    exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                    self.gh.update_node_to_implicit_executor(node_id, exec_id)
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE_LOOP, 'time': t})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)

            elif cwu.is_match(token.astParent, '.') and token.astParent.astOperand1:
                exec_token = token.astParent.astOperand1
                exec_id = self.get_executor_val_id(exec_token)                
                if exec_id:
                    if token.str == 'spin_node_all':
                        node_token = cwu.get_comma_separated_token_at_pose_from_right(arg_cnt)
                        node_id = cwu.get_val_id(node_token)
                        if node_id:
                            # STORE NODE TO (DEFAULT) EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                            self.gh.update_node_to_implicit_executor(node_id, exec_id)
                            self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                            ##TODO add control edges for spin to detect wrong location of spin invocation?
                            spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE_LOOP, 'time': t})
                            self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                            token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                            self.track_thr_token(token)

                            self.track_spin_locations(token)
                    elif token.str == 'spin_all':        
                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE_LOOP, 'time': t})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                        self.track_thr_token(token)

                        self.track_spin_locations(token)


    ''' The following functions with rclcpp:: prefix do add_node and remove_node: ##TODO support remove_node?
        rclcpp::spin_once(timeout=-1) NOT YET IMPLEMENTED
        executor.spin_node_once(node, timeout=-1) # milli default for std::chrono::duration
        executor.spin_once(timeout=-1) # ns
        '''
    def process_spin_once(self, token, left_token, right_token):
        if token.str.startswith('spin') and token.str.endswith('once') and cwu.is_match(token.next, '('):
            paren_token = token.next
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            t = -1 # block indefinitely

            if cwu.is_match(token.astParent, '.') and token.astParent.astOperand1:
                exec_token = token.astParent.astOperand1
                exec_id = self.get_executor_val_id(exec_token)                
                if exec_id:
                    if token.str == 'spin_node_once':
                        node_token = cwu.get_comma_separated_token_at_pose_from_right(arg_cnt)
                        node_id = cwu.get_val_id(node_token)
                        if node_id:
                            if arg_cnt == 2:
                                t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                                t = cwu.get_time_value(t_arg)

                            # STORE NODE TO (DEFAULT) EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                            self.gh.update_node_to_implicit_executor(node_id, exec_id)
                            self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)                            

                            ##TODO add control edges for spin to detect wrong location of spin invocation?
                            spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.ONE, 'time': t})
                            self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                            token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                            self.track_thr_token(token)

                            self.track_spin_locations(token)
                    elif token.str == 'spin_once':
                        if arg_cnt == 1:
                            t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                            t = cwu.get_time_value(t_arg)

                        ##TODO add control edges for spin to detect wrong location of spin invocation?
                        spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.ONE, 'time': t})
                        self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                        token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                        self.track_thr_token(token)

                        self.track_spin_locations(token)


    ''' The following functions with rclcpp:: prefix do add_node and remove_node: ##TODO support remove_node?
        rclcpp::spin_until_future_complete(node,future,timeout=-1) # milli default for std::chrono::duration
        rclcpp::executors::spin_node_until_future_complete(executor,node,future,timeout=-1) # milli default for std::chrono::duration
        executor.spin_until_future_complete(future,timeout=-1) # milli default for std::chrono::duration
        '''
    def process_spin_until_future_complete(self, token, left_token, right_token):
        if token.str.startswith('spin') and token.str.endswith('until_future_complete') and cwu.is_match(token.next, '('):
            paren_token = token.next
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            t = -1  # block indefinitely
            cbg_id = None

            if cwu.is_match_previous(token, ['rclcpp', '::']):
                node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                node_id = cwu.get_val_id(node_token)      
                if node_id:
                    if arg_cnt == 3:
                        t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        t = cwu.get_time_value(t_arg)

                    # STORE NODE TO DEFAULT EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                    exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                    self.gh.update_node_to_implicit_executor(node_id, exec_id)
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.FUTURE, 'time': t})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)

                    fu_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                    cbg_id = cwu.get_val_id(fu_arg)

            elif cwu.is_match_previous(token, ['executors', '::']):
                node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                node_id = cwu.get_val_id(node_token)
                exec_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                exec_id = self.get_executor_val_id(exec_token)
                if node_id and exec_id:
                    if arg_cnt == 4:
                        t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        t = cwu.get_time_value(t_arg)

                    # STORE NODE TO (DEFAULT) EXECUTOR MAPPING TO FIND THE EXECUTOR FOR THE NODE'S CALLBACK GROUPS?
                    self.gh.update_node_to_implicit_executor(node_id, exec_id)
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN) 

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.FUTURE, 'time': t})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)

                    fu_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                    cbg_id = cwu.get_val_id(fu_arg)

            elif cwu.is_match(token.astParent, '.') and token.astParent.astOperand1:
                exec_token = token.astParent.astOperand1
                exec_id = self.get_executor_val_id(exec_token)                
                if exec_id:
                    if arg_cnt == 2:
                        t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        t = cwu.get_time_value(t_arg)

                    ##TODO add control edges for spin to detect wrong location of spin invocation?
                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.FUTURE, 'time': t})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)

                    fu_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                    cbg_id = cwu.get_val_id(fu_arg)


            if cbg_id:
                cb_id = self.gh.add_graph_node('future_done_cb', self.gh.NType.FUTURE, {'cb_type': self.gh.CbFunc.OTHER, 'implicit': True, 'time': t})
                
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                self.gh.track_callback_function((None, cb_id), cb_id)

                token.val = cwu.update_val_dict(token.val, 'id', cb_id)
                self.track_cb_link_token(token)


    ''' node.create_subscription(topic,qos,callback,options=suboptions(),msgmemstrat=default)
        options.callback_group=nullptr
        '''
    def process_create_subscription(self, token, left_token, right_token):
        if token.str == 'create_subscription':
            paren_token = None
            node_id = ''
            if cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                node_token = token.astParent.astOperand1
                node_id = cwu.get_val_id(node_token)
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                node_id = self.get_node_id_for_token(token)

            if node_id:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                cb_id = None
                cbg_id = None

                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SUB})
                    self.gh.track_callback_function(cb_func, cb_id)

                cbg_token = None
                if arg_cnt == 4:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                elif arg_cnt == 5:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2)
                
                if cbg_token:
                    cbg_id = cwu.get_val_id(cbg_token)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)


    ''' node.create_wall_timer(period,callback,group=nullptr)
        '''
    def process_create_wall_timer(self, token, left_token, right_token):
        if token.str == 'create_wall_timer':
            paren_token = None
            node_id = ''
            if cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                node_token = token.astParent.astOperand1
                node_id = cwu.get_val_id(node_token)
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                node_id = self.get_node_id_for_token(token)

            if node_id:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                cb_id = None
                cbg_id = None

                if arg_cnt == 3:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2)
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cbg_id = cwu.get_val_id(cbg_token)
                elif arg_cnt == 2:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    t_arg = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                    t = cwu.get_time_value(t_arg)
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.TIMER, 'time': t})
                    self.gh.track_callback_function(cb_func, cb_id)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)
                    
                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)


    ''' node.create_client(service,qos_profile=default,group=nullptr)
        '''
    def process_create_client(self, token, left_token, right_token):
        if token.str == 'create_client':
            paren_token = None
            node_id = ''
            if cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                node_token = token.astParent.astOperand1
                node_id = cwu.get_val_id(node_token)
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                node_id = self.get_node_id_for_token(token)

            if node_id:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                cbg_id = None

                if arg_cnt == 3:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cbg_id = cwu.get_val_id(cbg_token)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)
                    
                if cbg_id:
                    # USE CBG WHILE CALLING SERVICE USING CLIENT
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                    self.disable_local_check = True


    ''' async_send_request(request)
        async_send_request(request, callback)
        '''
    def process_async_send_request(self, token, left_token, right_token):
        if token.str == 'async_send_request' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            cli_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(cli_token)

            ##TODO support future objects (future callback?). The following will propagage cb_group id to future object
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                self.disable_local_check = True

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                
            if arg_cnt == 2:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.CLI})
                    self.gh.track_callback_function(cb_func, cb_id)

                if cbg_id and cb_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


    ''' node.create_service(service,callback,qos_profile=default,group=nullptr)
        '''
    def process_create_service(self, token, left_token, right_token):
        if token.str == 'create_service':
            paren_token = None
            node_id = ''
            if cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
                paren_token = token.astParent.astParent
                node_token = token.astParent.astOperand1
                node_id = cwu.get_val_id(node_token)
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                node_id = self.get_node_id_for_token(token)

            if node_id:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                cb_id = None
                cbg_id = None

                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
                    self.gh.track_callback_function(cb_func, cb_id)

                cbg_token = None
                if arg_cnt == 4:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                
                if cbg_token:
                    cbg_id = cwu.get_val_id(cbg_token)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)


    ''' rclcpp_action::create_server(node,action,goal_callback,cancel_callback,accepted_callback,options=default,group=nullptr)
        '''
    def process_create_action_server(self, token, left_token, right_token):
        if token.str == 'create_server' and cwu.is_match_previous(token, ['rclcpp_action', '::']) and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
            node_id = cwu.get_val_id(node_token)
            if node_id:
                cbd_id = None
                i = 0 #COUNTER TO HANDLE DEFAULT ARGS

                if arg_cnt == 7:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cbg_id = cwu.get_val_id(cbg_token)
                    i = 2
                elif arg_cnt == 6:
                    i = 1

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    cb3_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1+i) #ACCEPTED CB
                    cb2_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2+i) #CANCEL CB
                    cb1_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 3+i) #GOAL CB

                    if cb1_token:
                        cb1_func = cwu.get_callback_function(cb1_token)
                        if cb1_func:
                            cb1_id = self.gh.add_graph_node(cb1_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV})
                            if cb1_id:
                                self.gh.add_graph_edge(cbg_id, cb1_id, self.gh.EType.CONTAIN)

                    if cb2_token:
                        cb2_func = cwu.get_callback_function(cb2_token)
                        if cb2_func:
                            cb2_id = self.gh.add_graph_node(cb2_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV})
                            if cb2_id:
                                self.gh.add_graph_edge(cbg_id, cb2_id, self.gh.EType.CONTAIN)

                    if cb3_token:
                        cb3_func = cwu.get_callback_function(cb3_token)
                        if cb3_func:
                            cb3_id = self.gh.add_graph_node(cb3_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV})
                            if cb3_id:
                                self.gh.add_graph_edge(cbg_id, cb3_id, self.gh.EType.CONTAIN)

                    ###TODO set callbacks as id?

                    
    ''' rclcpp_action::create_client(node,action,group=nullptr) 
	    '''
    def process_create_action_client(self, token, left_token, right_token):
        if token.str == 'create_client' and cwu.is_match_previous(token, ['rclcpp_action', '::']) and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
            node_id = cwu.get_val_id(node_token)
            if node_id:
                cbg_id = None

                if arg_cnt == 3:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cbg_id = cwu.get_val_id(cbg_token)

                if not cbg_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                    self.disable_local_check = True


    ''' options.goal_response_callback = cb
        options.feedback_callback = cb
        options.result_callback = cb
        '''
    def process_sendgoaloptions(self, token, left_token, right_token):
        if token.str in ['goal_response_callback', 'feedback_callback', 'result_callback'] and \
                cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('SendGoalOptions'):
                cb_token = token.astParent.astParent.astOperand2
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    if token.str.startswith('g'):
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'g_cb', cb_func)
                    elif token.str.startswith('f'):
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'f_cb', cb_func)
                    elif token.str.startswith('r'):
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'r_cb', cb_func)


    ''' action_client.async_send_goal(goal,options=SendGoalOptions())
        '''
    def process_async_send_goal(self, token, left_token, right_token):
        if token.str == 'async_send_goal' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            cli_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(cli_token)

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 2:
                opt_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
                if opt_var and opt_var.variable and opt_var.variable.val:
                    g_cb_func = opt_var.variable.val.get('g_cb')
                    f_cb_func = opt_var.variable.val.get('f_cb')
                    r_cb_func = opt_var.variable.val.get('r_cb')

                    ##TODO add control edges between callback functions e.g. from goal_response_callback to result_callback?

                    if g_cb_func:
                        g_cb_id = self.gh.add_graph_node(g_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_CLI})
                        self.gh.track_callback_function(g_cb_func, g_cb_id)
                        if g_cb_id and cbg_id:
                            self.gh.add_graph_edge(cbg_id, g_cb_id, self.gh.EType.CONTAIN)

                    if f_cb_func:
                        f_cb_id = self.gh.add_graph_node(f_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                        self.gh.track_callback_function(f_cb_func, f_cb_id)
                        if f_cb_id and cbg_id:
                            self.gh.add_graph_edge(cbg_id, f_cb_id, self.gh.EType.CONTAIN)

                    if r_cb_func:
                        r_cb_id = self.gh.add_graph_node(r_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_CLI})
                        self.gh.track_callback_function(r_cb_func, r_cb_id)
                        if r_cb_id and cbg_id:
                            self.gh.add_graph_edge(cbg_id, r_cb_id, self.gh.EType.CONTAIN)

            # The following will propagage cb_group id to future object
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                self.disable_local_check = True
                              

    ''' action_client.async_get_result(goal_handle,result_callback=nullptr)
        If used, overrides the result_callback set previously e.g. via sendgoaloptions
        '''
    def process_async_get_result(self, token, left_token, right_token):
        if token.str == 'async_get_result' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            cli_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(cli_token)

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 2:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_CLI})
                    self.gh.track_callback_function(cb_func, cb_id)
                    if cb_id and cbg_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

            # The following will propagage cb_group id to future object
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                self.disable_local_check = True


    ''' action_client.async_cancel_goal(goal_handle,cancel_callback=nullptr)
        action_client.async_cancel_goals_before(rclcpp_time_stamp,cancel_callback=nullptr)
        action_client.async_cancel_all_goals(cancel_callback=nullptr)
        '''
    def process_async_cancel_goal(self, token, left_token, right_token):
        if token.str.startswith('async_cancel_') and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            cli_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(cli_token)

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if ((token.str.endswith('goal') or token.str.endswith('goals_before')) and arg_cnt == 2) or \
                    (token.str.endswith('all_goals') and arg_cnt == 1):
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_CLI})
                    self.gh.track_callback_function(cb_func, cb_id)
                    if cb_id and cbg_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

            # The following will propagage cb_group id to future object
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                self.disable_local_check = True


    ''' node.add_on_set_parameters_callback(callback)
        '''
    def process_add_on_set_parameters_callback(self, token, left_token, right_token):
        if token.str == 'add_on_set_parameters_callback' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            node_id = cwu.get_val_id(node_token)

            cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

            cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
            cb_func = cwu.get_callback_function(cb_token)
            cb_id = None
            if cb_func:
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
                self.gh.track_callback_function(cb_func, cb_id)

            if cb_id and cbg_id:
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


    # message filter Subscriber and Synchronizer
    ##TODO empty constructor and subscribe, empty constructor and connectInput
    ''' Subscriber(NodeHandle, topic, queue_size, transport_hints=ros::TransportHints(), callback_queue=0)
        Synchronizer(x), where x is 2 to 9 inputs
        Synchronizer(Policy, x), where x is 2 to 9 inputs
        Subscriber.subscribe(NodeHandle, topic, queue_size, transport_hints=ros::TransportHints(), callback_queue=0)
        '''
    def process_message_filter_subscriber_synchronizer(self, token, left_token, right_token):
        sub_token = None
        if token.previous and token.previous.str == '>' and token.previous.link and \
                token.previous.link.previous and token.previous.link.previous.str == 'Subscriber':
            sub_token = token
        elif token.str == 'subscribe' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            sub_token = token.astParent.astOperand1
            
        if sub_token:
            paren_token = None
            
            (sub_var, sub_str) = cwu.get_var_token_and_var_name(sub_token)
            if sub_var and sub_var.variable:
                if 'Subscriber' not in cwu.find_sanitized_variable_type(sub_var.variable):
                    return
                if sub_token.astParent and sub_token.astParent.str == '(':
                    paren_token = sub_token.astParent
                elif sub_token.astParent and sub_token.astParent.astParent and sub_token.astParent.astParent.str == '(':
                    paren_token = sub_token.astParent.astParent
            elif sub_token.str == '(':
                paren_token = sub_token

            if paren_token:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                cbg_id = None

                if arg_cnt == 5:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    if cbg_token:
                        cbg_id = cwu.get_val_id(cbg_token)

                if not cbg_id and arg_cnt >= 1:
                    node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                    node_id = cwu.get_val_id(node_token)
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

                if cbg_id:
                    if sub_var and sub_var.variable:
                        sub_var.variable.val = cwu.update_val_dict(sub_var.variable.val, 'cbg', cbg_id)
                    else:
                        paren_token.val = cwu.update_val_dict(paren_token.val, 'cbg', cbg_id)

        elif token.previous and token.previous.str == '>' and token.previous.link and token.previous.link.previous:
            ty_token = token.previous.link.previous
            paren_token = None
            sync_token = None

            if ty_token.str.endswith('Synchronizer'):
                sync_token = token

                if sync_token.str == '(':
                    paren_token = sync_token
                    sync_token = None
                elif sync_token.astParent and sync_token.astParent.str == '(':
                    paren_token = sync_token.astParent
                elif sync_token.astParent and sync_token.astParent.astParent and sync_token.astParent.astParent.str == '(':
                    paren_token = sync_token.astParent.astParent

            elif ty_token.str in ['make_shared', 'make_unique'] and ty_token.val:
                type_token = ty_token.val.get('type_token')
                if type_token and type_token.str.endswith('Synchronizer'):
                    paren_token = ty_token.val.get('paren_token')

            if paren_token:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                if arg_cnt >= 1:
                    cbg_set = set()

                    while arg_cnt > 0:
                        arg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                        (arg_var, arg_str) = cwu.get_var_token_and_var_name(arg_token)
                        if arg_var and arg_var.variable and arg_var.variable.val:
                            arg_type = cwu.find_sanitized_variable_type(arg_var.variable)
                            if arg_type and any([arg_type.startswith(s) for s in ['message_filters::', 'Subscriber']]):
                                cbg_id = arg_var.variable.val.get('cbg')
                                if cbg_id:
                                    cbg_set.add(cbg_id)
                        arg_cnt = arg_cnt - 1

                    if cbg_set:
                        if sync_token:
                            (sync_var, sync_str) = cwu.get_var_token_and_var_name(sync_token)
                            if sync_var and sync_var.variable:
                                sync_var.variable.val = cwu.update_val_dict(sync_var.variable.val, 'cbg_set', cbg_set)
                        
                        paren_token.val = cwu.update_val_dict(paren_token.val, 'cbg_set', cbg_set)

        elif token.str == 'registerCallback' and token.astParent and token.astParent.str == '.' and \
                token.astParent.astParent and token.astParent.astParent.str == '(':
                
            if token.astParent.astOperand1:
                obj_token = token.astParent.astOperand1
                (obj_var, obj_str) = cwu.get_var_token_and_var_name(obj_token)
                if obj_var and obj_var.variable and obj_var.variable.val:
                    cbg_id = obj_var.variable.val.get('cbg_set')
                    if not cbg_id:
                        cbg_id = obj_var.variable.val.get('cbg')

                    if cbg_id:
                        paren_token = token.astParent.astParent
                        arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                        if arg_cnt >= 1:
                            cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                            cb_func = cwu.get_callback_function(cb_token)
                            if cb_func:
                                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.OTHER})
                                self.gh.track_callback_function(cb_func, cb_id)
                                    
                                if isinstance(cbg_id, set):
                                    for cbg in cbg_id:
                                        self.gh.add_graph_edge(cbg, cb_id, self.gh.EType.CONTAIN)
                                else:
                                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)

                                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)
                                if cwu.is_local_symbol(obj_var.variable):
                                    self.track_local_token(paren_token)


    def process_while(self, token, left_token, right_token):
        if token.str == 'while' and cwu.is_match(token.astParent, '(') and cwu.is_match(token.astParent.astOperand2, '('):
            test_token = token.astParent.astOperand2
            if cwu.is_match_previous(test_token, ['rclcpp', '::', 'ok']):
                self.track_spin_locations(token)


    def process_lifecycle_callbacks(self, function_object):
        if function_object and \
         function_object.name in ['on_configure', 'on_activate', 'on_deactivate', 'on_cleanup', 'on_shutdown', 'on_error'] and \
                function_object.isImplicitlyVirtual:
            node_id = cwu.get_class_scope_name_for_function(function_object)

            cbg_id = self.gh.get_default_cbgroup_of_node(node_id)

            cb_func = (function_object, node_id + '::' + function_object.name)
            cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
            self.gh.track_callback_function(cb_func, cb_id)

            if cb_id and cbg_id:
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


    def process_main(self, function_object):
        if function_object and function_object.name == 'main':
            thr_func = (function_object, function_object.name)
            thr_id = self.gh.setup_thread(1)
            self.gh.track_thread_function(thr_func, thr_id)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # the ROS API will be called without a ros node objects if called in a class derived from rclcpp node
    # get node id based on that class'es name
    def get_node_id_for_token(self, token):
        node_id = ''
        if token.scope and token.scope.type == 'Function' and token.scope.function:
            func_def_token = token.scope.function.tokenDef
            if func_def_token.scope and func_def_token.scope.type == 'Class':
                node_id = func_def_token.scope.className
                self.gh.update_node(node_id)
        return node_id
                

    def get_executor_val_id(self, exec_token):
        exec_id = ''
        (exec_var, exec_str) = cwu.get_var_token_and_var_name(exec_token)
        if exec_var and exec_var.variable and exec_var.variable.val:
            exec_id = exec_var.variable.val.get('id', '')
        elif exec_token.val:
            exec_id = exec_token.val.get('id', '')

        if not exec_id:
            # IF THERE IS NO AST FOR VARIABLE DECLARATION, EXECUTOR GRAPH NODE IS NOT THERE
            if exec_var and exec_var.variable:
                thr_cnt = None

                exec_type = cwu.find_sanitized_variable_type(exec_var.variable)
                if exec_type.endswith('MultiThreadedExecutor'):
                    thr_cnt = self.gh.ThrCount.CORES
                elif exec_type.endswith('SingleThreadedExecutor'): # ALSO COVERS StaticSingleThreadedExecutor
                    thr_cnt = 1
                    # thr_id same as spin call
                elif exec_type.endswith('Executor'):
                    thr_cnt = self.gh.ThrCount.UNKNOWN

                if thr_cnt:
                    thr_id = self.gh.setup_thread(thr_cnt, setup_last_thr_id=False)

                    # EXECUTOR GRAPH NODE HAS NOT BEEN ADDED TO THE GRAPH; ADD ONE
                    exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': thr_cnt, 'thr_id': thr_id})
                    exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'id', exec_id)
                    exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'thr_count', thr_cnt)
                    exec_var.variable.val = cwu.update_val_dict(exec_var.variable.val, 'thr_id', thr_id)

        return exec_id
    
    
    ##TODO refactor code to get_node_val_id, get_callback_group (similar to cpp1_walker) instead of setup param of graph_helper functions


    def construct_node(self, paren_token, node_token):
        node_id = ''
        # typeScope is not there when the node is created in another package; So, disable the check for now
        if node_token: #and node_token.typeScope and node_token.typeScope.type == 'Class':
            node_id = node_token.str

        if paren_token and node_id:
            paren_token.val = cwu.update_val_dict(paren_token.val, 'is_active', True)
            paren_token.val = cwu.update_val_dict(paren_token.val, 'id', node_id)
                
            self.gh.update_node(node_id, is_active=True, setup=False)

        return node_id
    

    def construct_executor(self, paren_token, exec_type_token):
        exec_id = ''
        thr_cnt = None
        thr_id = None

        if paren_token and exec_type_token:
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if exec_type_token.str.endswith('SingleThreadedExecutor'):
                thr_cnt = 1
                # thr_id same as spin call
            elif exec_type_token.str.endswith('MultiThreadedExecutor'):
                if arg_cnt in [0, 1]: ##TODO: check if 1 == cores??
                    thr_cnt = self.gh.ThrCount.CORES        
                else:
                    thr_arg = cwu.get_comma_separated_token_at_pose_from_right(arg_cnt-1)
                    thr_cnt = cwu.get_numeric_value(thr_arg)
                    # one thr_id is same as the spin call and the remaining are the new ids

            if thr_cnt:
                thr_id = self.gh.setup_thread(thr_cnt, setup_last_thr_id=False)
                exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': thr_cnt, 'thr_id': thr_id})

        return (exec_id, thr_cnt, thr_id)
    

    def construct_thread(self, paren_token):
        thr_id = ''

        if paren_token:
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            if arg_cnt >= 1:
                func_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                thr_func = cwu.get_thread_function(func_token)
                if thr_func:
                    thr_id = self.gh.setup_thread(1)
                    self.gh.track_thread_function(thr_func, thr_id)

        return thr_id


    def track_cb_link_token(self, token):
        self.cb_link_track_tokens.append(token)

    
    def get_cb_link_track_items(self):
        cb_track_items = []
        for t in self.cb_link_track_tokens:
            cb_track_items.append((cwu.get_function_for_token(t), t.val['id']))
        return cb_track_items
    

    def clear_cb_link_track_tokens(self):
        self.cb_link_track_tokens.clear()

    
    def track_thr_token(self, token):
        self.thr_track_tokens.append(token)


    def get_thr_track_items(self):
        thr_track_items = []
        for t in self.thr_track_tokens:
            thr_track_items.append((cwu.get_function_for_token(t), t.val['id']))
        return thr_track_items
    

    def clear_thr_track_tokens(self):
        self.thr_track_tokens.clear()


    def track_local_token(self, token):
        self.local_track_tokens.append(token)


    def get_local_track_items(self):
        local_track_items = []
        for l in self.local_track_tokens:
            local_track_items.append((cwu.get_function_for_token(l), l.val['id']))
        return local_track_items
    

    def clear_local_track_tokens(self):
        self.local_track_tokens.clear()


    def track_spin_locations(self, token):
        self.spin_locations.append(cwu.get_function_for_token(token))


    def get_spin_locations(self):
        return self.spin_locations


    def clear_spin_locations(self):
        self.spin_locations.clear()
