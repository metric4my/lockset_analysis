#Copyright 2025 Purdue University.
#All rights reserved.


import cpp_walker_utils as cwu
from graph_helper import GraphHelper


class CPP1Walker:
    
    def __init__(self, gh) -> None:
        self.gh = gh #GraphHelper()

        self.was_some_value_changed = False
        self.found_values_in_this_tree = False
        self.disable_local_check = False

        self.current_node_id = None # FOR ROS1

        self.thr_track_tokens = [] # tokens tracking the potential processing in a new thread
        self.cb_link_track_tokens= [] # tokens tracking the potential cb-cb invoke links
        self.local_track_tokens = [] # tokens tracking the local declrations of node, subscriber, timer, client, server, etc.
        self.spin_locations = [] # functions that call spin



    
    def collect(self, root_token):
        cwu.generic_recurse_and_apply_function(root_token, self.process_thread_construction)
        
        cwu.generic_recurse_and_apply_function(root_token, self.process_ros_init)
        cwu.generic_recurse_and_apply_function(root_token, self.process_node_handle)

        cwu.generic_recurse_and_apply_function(root_token, self.process_callback_queue)
        cwu.generic_recurse_and_apply_function(root_token, self.process_get_callback_queue)
        cwu.generic_recurse_and_apply_function(root_token, self.process_set_callback_queue)

        cwu.generic_recurse_and_apply_function(root_token, self.process_subscribeoptions)
        cwu.generic_recurse_and_apply_function(root_token, self.process_timeroptions)
        cwu.generic_recurse_and_apply_function(root_token, self.process_advertiseoptions)
        cwu.generic_recurse_and_apply_function(root_token, self.process_advertiseserviceoptions)
        cwu.generic_recurse_and_apply_function(root_token, self.process_options_callback_queue)
        cwu.generic_recurse_and_apply_function(root_token, self.process_subscribe)
        cwu.generic_recurse_and_apply_function(root_token, self.process_create_timer)
        cwu.generic_recurse_and_apply_function(root_token, self.process_advertise)
        cwu.generic_recurse_and_apply_function(root_token, self.process_advertise_service)
        #cwu.generic_recurse_and_apply_function(root_token, self.process_service_client)
        cwu.generic_recurse_and_apply_function(root_token, self.process_simple_action_client)
        cwu.generic_recurse_and_apply_function(root_token, self.process_send_goal)
        cwu.generic_recurse_and_apply_function(root_token, self.process_simple_action_server)
        cwu.generic_recurse_and_apply_function(root_token, self.process_register_goal_callback)
        cwu.generic_recurse_and_apply_function(root_token, self.process_register_result_callback)
        cwu.generic_recurse_and_apply_function(root_token, self.process_message_filter_subscriber_synchronizer)

        cwu.generic_recurse_and_apply_function(root_token, self.process_copy_construct)
        cwu.generic_recurse_and_apply_function(root_token, self.process_assignment)
        cwu.generic_recurse_and_apply_function(root_token, self.process_while)

        cwu.generic_recurse_and_apply_function(root_token, self.process_spinner)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin)
        cwu.generic_recurse_and_apply_function(root_token, self.process_async_spinner_start)
        cwu.generic_recurse_and_apply_function(root_token, self.process_spin_once)


    def collect_from_function(self, function_object):
        self.process_main(function_object)
    

    def reset_current_node_id(self):
        self.current_node_id = None


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


    ''' obj1(obj2) where T(obj1)==T(obj2) '''
    def process_copy_construct(self, token, left_token, right_token):
        if token.str == '(' and left_token and right_token:
            (left_var, left_str) = cwu.get_var_token_and_var_name(left_token)
            if left_var and left_var.variable:
                (right_var, right_str) = cwu.get_var_token_and_var_name(right_token)
                if right_var and right_var.variable:
                    if cwu.find_sanitized_variable_type(left_var.variable) == cwu.find_sanitized_variable_type(right_var.variable):
                        left_var.variable.val = right_var.variable.val

    ''' boost/std::thread th(func, args) => args by value or ref
        boost/std::thread th(&func, &obj) => obj.func
        '''
    def process_thread_construction(self, token, left_token, right_token):
        if token.str == 'thread':
            paren_token = None

            if cwu.is_match(token.previous, '::') and cwu.is_match(token.previous.astParent, '('):
                paren_token = token.previous.astParent
                thr_token = token.previous.astOperand2
            elif cwu.is_match(token.astParent, '('):
                paren_token = token.astParent
                thr_token = token.next

            if paren_token:
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                if arg_cnt >= 1:
                    func_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                    thr_func = cwu.get_thread_function(func_token)
                    if thr_func:
                        thr_id = self.gh.setup_thread(1)
                        self.gh.track_thread_function(thr_func, thr_id)

                        (thr_var, thr_str) = cwu.get_var_token_and_var_name(thr_token)
                        if thr_var and thr_var.variable:
                            thr_var.variable.val = cwu.update_val_dict(thr_var.variable.val, 'id', thr_id)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # ROS 1
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

    ''' ros::init(argc, argv, "my_node_name")
        ros::init(argc, argv, "my_node_name", ros::init_options::AnonymousName)    
        '''
    def process_ros_init(self, token, left_token, right_token):
        if token.str == 'init' and cwu.is_match_previous(token, ['ros', '::']) and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            node_token = None
            if arg_cnt == 3:
                node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
            elif arg_cnt == 4:
                node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2)

            if node_token and node_token.isString:
                node_id = node_token.str
                if self.current_node_id and (node_id != self.current_node_id):
                    self.gh.relabel_graph_node(self.current_node_id, node_id)
                else:
                    self.gh.setup_and_get_node(node_id)

                self.gh.update_node(node_id, is_active=True, setup=False)
                self.current_node_id = node_id

                node_token.val = cwu.update_val_dict(node_token.val, 'id', (node_id,))
                self.track_thr_token(node_token)
                

    ''' ros::NodeHandle nh(ns=std::string(),remappings=M_string()) => global cbg
        ros::NodeHandle nh(rhs_nh) => rhs_nh cbg => TODO handle with copy constructor?
        ros::NodeHandle nh(parent_nh,ns) => parent_nh cbg
        ros::NodeHandle nh(parent_nh,ns,remappings) => parent_nh cbg
        '''
    def process_node_handle(self, token, left_token, right_token):
        if token.str == 'NodeHandle' and cwu.is_match_previous(token, ['ros', '::']):
            node_token = token.previous.astOperand2
            if node_token:
                # UPDATES THE CURRENT NODE ID AND NODE VAL ID; EACH NODE HANDLE HAS THE SAME ID
                node_id = self.get_node_val_id(node_token)
                if node_id and cwu.is_match(token.previous.astParent, '('):
                    paren_token = token.previous.astParent
                    arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                    # PROPAGATE CALLBACK QUEUE BETWEEN NODE HANDLES
                    if arg_cnt >= 1:
                        handle_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                        (handle_var, handle_str) = cwu.get_var_token_and_var_name(handle_token)
                        if handle_var and handle_var.variable and cwu.find_sanitized_variable_type(handle_var.variable).endswith('NodeHandle'):
                            cbg_id = handle_var.variable.val.get('cbg') if handle_var.variable.val else None
                            if cbg_id:
                                (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
                                if node_var and node_var.variable:
                                    node_var.variable.val = cwu.update_val_dict(node_var.variable.val, 'cbg', cbg_id)

                ### how to check if node_id is local?


    ''' ros::CallbackQueue(enabled=true)
        '''
    def process_callback_queue(self, token, left_token, right_token):
        if token.str == 'CallbackQueue' and cwu.is_match_previous(token, ['ros', '::']):
            cbg_id = None

            cbg_token = token.previous.astOperand2
            (cbg_var, cbg_str) = cwu.get_var_token_and_var_name(cbg_token)
            if cbg_var and cbg_var.variable:
                cbg_id = self.gh.add_graph_node('cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                cbg_var.variable.val = cwu.update_val_dict(cbg_var.variable.val, 'id', cbg_id)
                                    
            if cwu.is_match(token.previous.astParent, '('):
                paren_token = token.previous.astParent
                if not cbg_id:
                    cbg_id = self.gh.add_graph_node('cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)


    ''' callbacks from advertisements/subscriptions/services/etc. are added to the specified queue. 
        NULL (by default) uses the global queue.
        nh.setCallbackQueue(queue)
        '''
    def process_set_callback_queue(self, token, left_token, right_token):
        if token.str == 'setCallbackQueue' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
            if node_var and node_var.variable:
                node_id = self.update_and_get_node_id(node_var.variable)

                cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                cbg_id = cwu.get_val_id(cbg_token)

                if node_id and cbg_id:
                    # STORE CALLBACK QUEUE IN NODE HANDLE, AS EACH HANDLE CAN HAVE IT'S OWN QUEUE
                    node_var.variable.val = cwu.update_val_dict(node_var.variable.val, 'cbg', cbg_id)
                    self.gh.add_graph_edge(node_id, cbg_id, self.gh.EType.CONTAIN)


    ''' returns cbg
        nh.getCallbackQueue()
        '''
    def process_get_callback_queue(self, token, left_token, right_token):
        if token.str == 'getCallbackQueue' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            cbg_id = self.get_node_callback_group(node_token)
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
                self.disable_local_check = True


    def process_options_callback_queue(self, token, left_token, right_token):
        if token.str == 'callback_queue' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable:
                opt_type = cwu.find_sanitized_variable_type(opt_var.variable)
                if opt_type.endswith('SubscribeOptions') or opt_type.endswith('AdvertiseOptions') or \
                        opt_type.endswith('TimerOptions') or opt_type.endswith('AdvertiseServiceOptions'):
                    cbg_token = token.astParent.astParent.astOperand2
                    cbg_id = cwu.get_val_id(cbg_token)
                    if cbg_id:
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cbg', cbg_id)


    ''' SubscribeOptions.callback_queue = cbg (processed above)
        SubscribeOptions.allow_concurrent_callbacks = true/false
        SubscribeOptions.template init(_topic,_queue_size,_callback,factory_fn=DefaultMessageCreator)
        '''
    def process_subscribeoptions(self, token, left_token, right_token):
        if token.str == 'allow_concurrent_callbacks' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('SubscribeOptions'):
                conc_token = token.astParent.astParent.astOperand2
                conc = cwu.get_numeric_value(conc_token)
                opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'concurrent', conc)

        elif token.str == 'template' and cwu.is_match_next(token, ['init' '(']) and cwu.is_match(token.next.astParent, '.'):
            opt_token = token.next.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('SubscribeOptions'):
                paren_token = token.next.next
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                if arg_cnt in [3,4]:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cb', cb_func)


    ''' returns subscriber
        nh.subscribe(topic,queue_size,callback,obj,transport_hints=TransportHints())
        nh.subscribe(topic,queue_size,callback,transport_hints=TransportHints())
        nh.subscribe(topic,queue_size,callback,tracked_object=VoidConstPtr(),transport_hints=TransportHints())
        nh.subscribe(SubscribeOptions)
        '''
    def process_subscribe(self, token, left_token, right_token):
        if token.str == 'subscribe' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
            if node_var and node_var.variable:
                if not cwu.find_sanitized_variable_type(node_var.variable).endswith('NodeHandle'):
                    return

            cbg_id = None
            cb_id = None

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 1:
                opt_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
                if opt_var and opt_var.variable and opt_var.variable.val:
                    cbg_id = opt_var.variable.val.get('cbg')
                    cb_func = opt_var.variable.val.get('cb')
                    if cb_func:
                        conc = opt_var.variable.val.get('concurrent', False)
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SUB, 'concurrent': conc})
                        self.gh.track_callback_function(cb_func, cb_id)
                        
            ##TODO get the token for the object whose member functions is a callback
            elif arg_cnt in [3,4,5]:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SUB, 'concurrent': False})
                    self.gh.track_callback_function(cb_func, cb_id)

            if not cbg_id:
                cbg_id = self.get_node_callback_group(node_token)

            if cb_id and cbg_id:
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)



    ''' AdvertiseOptions.callback_queue = cbg (processed above)
        AdvertiseOptions.connect_cb = cb
        AdvertiseOptions.disconnect_cb = cb
        AdvertiseOptions.template init(_topic,_queue_size,_connect_cb=default,_disconnect_cb=default)
        '''
    def process_advertiseoptions(self, token, left_token, right_token):
        if token.str in ['connect_cb', 'disconnect_cb'] and \
                cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('AdvertiseOptions'):
                cb_token = token.astParent.astParent.astOperand2
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    if token.str.startswith('c'):
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'c_cb', cb_func)
                    elif token.str.startswith('d'):
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'd_cb', cb_func)

        elif token.str == 'template' and cwu.is_match_next(token, ['init' '(']) and cwu.is_match(token.next.astParent, '.'):
            opt_token = token.next.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('AdvertiseOptions'):
                paren_token = token.next.next
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                
                if arg_cnt in [3,4]:
                    c_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                    c_cb_func = cwu.get_callback_function(c_cb_token)
                    if c_cb_func:
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'c_cb', c_cb_func)
                    
                    if arg_cnt == 4:
                        d_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        d_cb_func = cwu.get_callback_function(d_cb_token)
                        if d_cb_func:
                            opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'd_cb', d_cb_func)


    ''' returns publisher
        nh.advertise(topic,queue_size,latch=false)
        nh.advertise(topic,queue_size,connect_cb,disconnect_cb=SubscriberStatusCallback(),tracked_object=VoidConstPtr(),latch=false)
        nh.advertise(AdvertiseOptions)
        '''
    def process_advertise(self, token, left_token, right_token):
        if token.str == 'advertise' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            cbg_id = None
            c_cb_func = None
            d_cb_func = None

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 1:
                opt_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
                if opt_var and opt_var.variable and opt_var.variable.val:
                    cbg_id = opt_var.variable.val.get('cbg')
                    c_cb_func = opt_var.variable.val.get('c_cb')
                    d_cb_func = opt_var.variable.val.get('d_cb')

            elif arg_cnt >= 3:
                c_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                if c_cb_token.str in ['true', 'false'] or c_cb_token.isNumber:
                    return
                (c_cb_var, c_cb_str) = cwu.get_var_token_and_var_name(c_cb_token)
                if c_cb_var and c_cb_var.variable and cwu.find_sanitized_variable_type(c_cb_var.variable).endswith('bool'):
                    return
                c_cb_func = cwu.get_callback_function(c_cb_token)
                
                if arg_cnt > 3:
                    d_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-3)
                    d_cb_func = cwu.get_callback_function(d_cb_token)

            if not cbg_id:
                cbg_id = self.get_node_callback_group(node_token)

            if c_cb_func:
                c_cb_id = self.gh.add_graph_node(c_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.OTHER})
                self.gh.track_callback_function(c_cb_func, c_cb_id)
                if c_cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, c_cb_id, self.gh.EType.CONTAIN)
            if d_cb_func:
                d_cb_id = self.gh.add_graph_node(d_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.OTHER})
                self.gh.track_callback_function(d_cb_func, d_cb_id)
                if d_cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, d_cb_id, self.gh.EType.CONTAIN)


    ''' (Wall)TimerOptions.callback = cb
        (Wall)TimerOptions.callback_queue = cbg (processed above)
        (Wall)TimerOptions.period = t
        (Wall)TimerOptions(_period,_callback,_queue,oneshot=false,autostart=true)
        (Wall)TimerOptions default values: period=0.1, callback_queue=none, oneshot=false, autostart=true
        '''
    def process_timeroptions(self, token, left_token, right_token):
        ##TODO support oneshot and autostart

        if token.str == 'callback' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('TimerOptions'):
                cb_token = token.astParent.astParent.astOperand2
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cb', cb_func)

        elif token.str == 'period' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '='):
            opt_token = token.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('TimerOptions'):
                t_token = token.astParent.astParent.astOperand2
                t = cwu.get_time_value(t_token)
                opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'time', t)

        elif token.str.endswith('TimerOptions') and cwu.is_match_previous(token, ['ros', '::']) and cwu.is_match(token.previous.astParent, '('):
            paren_token = token.previous.astParent
            cbg_id = None
            cb_func = None
            t = None

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            if arg_cnt >= 3:
                cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                cbg_id = cwu.get_val_id(cbg_token)
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                cb_func = cwu.get_callback_function(cb_token)
                t_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                t = cwu.get_time_value(t_token)

            # UPDATE VARIABLE VAL IF ANY
            opt_token = token.previous.astOperand2
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable:
                if cbg_id:
                    opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cbg', cbg_id)
                if cb_func:
                    opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cb', cb_func)
                if t:
                    opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'time', t)

            # UPDATE PAREN VAL ANYWAY
            if cbg_id:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'cbg', cbg_id)
            if cb_func:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'cb', cb_func)
            if t:
                paren_token.val = cwu.update_val_dict(paren_token.val, 'time', t)
            

    ''' returns timer
        nh.createTimer(ros::Rate,callback,obj,oneshot=false,autostart=true)
        nh.createTimer(ros::Duration,callback,obj,oneshot=false,autostart=true)
        nh.createTimer(ros::Duration,callback,oneshot=false,autostart=true)
        nh.createTimer(TimerOptions)
        nh.createWallTimer(ros::WallDuration,callback,obj,oneshot=false,autostart=true)
        nh.createWallTimer(ros::WallDuration,callback,oneshot=false,autostart=true)
        nh.createWallTimer(WallTimerOptions)
        '''
    def process_create_timer(self, token, left_token, right_token):
        if token.str in ['createTimer', 'createWallTimer'] and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            cbg_id = None
            cb_func = None
            t = None

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 1:
                opt_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
                if opt_var and opt_var.variable and opt_var.variable.val:
                    cbg_id = opt_var.variable.val.get('cbg')
                    cb_func = opt_var.variable.val.get('cb')
                    t = opt_var.variable.val.get('t')

            ##TODO get the token for the object whose member functions is a callback
            elif arg_cnt >= 2:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                cb_func = cwu.get_callback_function(cb_token)
                t_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                t = cwu.get_time_value(t_token)

            if not t:
                t = 0.1 # DEFAULT VALUE

            if not cbg_id:
                cbg_id = self.get_node_callback_group(node_token)

            if cb_func:
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.TIMER, 'time': t})
                self.gh.track_callback_function(cb_func, cb_id)

            if cb_id and cbg_id:
                self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)


    ''' AdvertiseServiceOptions.callback_queue = cbg (processed above)
        AdvertiseServiceOptions.template init(_service,_callback)
        '''
    def process_advertiseserviceoptions(self, token, left_token, right_token):
        if token.str == 'template' and cwu.is_match_next(token, ['init' '(']) and cwu.is_match(token.next.astParent, '.'):
            opt_token = token.next.astParent.astOperand1
            (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
            if opt_var and opt_var.variable and cwu.find_sanitized_variable_type(opt_var.variable).endswith('AdvertiseServiceOptions'):
                paren_token = token.next.next
                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                if arg_cnt == 2:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        opt_var.variable.val = cwu.update_val_dict(opt_var.variable.val, 'cb', cb_func)


    ''' returns service server
        nh.advertiseService(service,callback,obj)
        nh.advertiseService(service,callback)
        nh.advertiseService(service,callback,tracked_obj=VoidConstPtr())
        nh.advertiseService(AdvertiseServiceOptions)
        '''
    def process_advertise_service(self, token, left_token, right_token):
        if token.str == 'advertiseService' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            node_token = token.astParent.astOperand1
            cbg_id = None
            cb_func = None

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)

            if arg_cnt == 1:
                opt_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                (opt_var, opt_str) = cwu.get_var_token_and_var_name(opt_token)
                if opt_var and opt_var.variable and opt_var.variable.val:
                    cbg_id = opt_var.variable.val.get('cbg')
                    cb_func = opt_var.variable.val.get('cb')

            elif arg_cnt in [2,3]:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                cb_func = cwu.get_callback_function(cb_token)

            if not cbg_id:
                cbg_id = self.get_node_callback_group(node_token)

            if cb_func:
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.SERV})
                self.gh.track_callback_function(cb_func, cb_id)
                if cb_id and cbg_id:
                    self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)
                    paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cb_id)


    ''' returns service client
        nh.serviceClient(service,persistent=false,header_values=M_string())
        nh.serviceClient(ServiceClientOptions)
        '''
    def process_service_client(self, token, left_token, right_token):
        pass


    ''' returns action client
        SimpleActionClient(name,spin_thread=true)
        SimpleActionClient(nodehandle,name,spin_thread=true)
        '''
    def process_simple_action_client(self, token, left_token, right_token):
        if token.str == 'SimpleActionClient' and cwu.is_match_previous(token, ['actionlib', '::']) and cwu.is_match(token.previous.astParent, '('):
            paren_token = token.previous.astParent
            cli_token = token.previous.astOperand2

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            new_thread_cbg = False
            node_token = None

            # SPIN THREAD = TRUE USES NEW THREAD TO SPIN ON THE NEW CALLBACK GROUP FOR ACTION CLIENT
            if arg_cnt == 1:
                new_thread_cbg = True
            elif arg_cnt == 2:
                first_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                (first_var, first_str) = cwu.get_var_token_and_var_name(first_token)
                if first_var and first_var.variable and cwu.find_sanitized_variable_type(first_var.variable).endswith('NodeHandle'):
                    new_thread_cbg = True
                    node_token = first_token
                else:
                    second_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    if cwu.get_numeric_value(second_token) == True:
                        new_thread_cbg = True
            elif arg_cnt == 3:
                node_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                third_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                if cwu.get_numeric_value(third_token) == True:
                    new_thread_cbg = True

            if node_token:
                node_id = self.get_node_val_id(node_token)
            else: # NODE_TOKEN = NULL
                if not self.current_node_id:
                    self.current_node_id = self.gh.setup_and_get_node()
                node_id = self.current_node_id

            if new_thread_cbg:
                exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
                self.gh.update_node_to_implicit_executor(node_id, exec_id, setup=False)

                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                # NO EDGE BETWEEN NODE AND CBG BECAUSE THERE IS DEDICATED EXECUTER FOR THAT CBG
                cbg_id = self.gh.add_graph_node('cbg', self.gh.NType.CALLBACK_GROUP, {'group_type': self.gh.CbGroup.FIFO})
                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)
            else:
                if node_token:
                    cbg_id = self.get_node_callback_group(node_token)
                else:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)

            (cli_var, cli_str) = cwu.get_var_token_and_var_name(cli_token)
            if cli_var and cli_var.variable:
                cli_var.variable.val = cwu.update_val_dict(cli_var.variable.val, 'id', cbg_id)
            
            paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
            self.disable_local_check = True


    ''' simple_action_client.sendGoal(goal,done_cb=default,active_cb=default,feedback_cb=default)
        '''
    def process_send_goal(self, token, left_token, right_token):
        if token.str == 'sendGoal' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            cli_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(cli_token)

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            if arg_cnt >= 2:
                d_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-1)
                d_cb_func = cwu.get_callback_function(d_cb_token)
                if d_cb_func:
                    d_cb_id = self.gh.add_graph_node(d_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                    self.gh.track_callback_function(d_cb_func, d_cb_id)
                    if cbg_id:
                        self.gh.add_graph_edge(cbg_id, d_cb_id, self.gh.EType.CONTAIN)

                if arg_cnt >= 3:
                    a_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt-2)
                    a_cb_func = cwu.get_callback_function(a_cb_token)
                    if a_cb_func:
                        a_cb_id = self.gh.add_graph_node(a_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                        self.gh.track_callback_function(a_cb_func, a_cb_id)
                        if cbg_id:
                            self.gh.add_graph_edge(cbg_id, a_cb_id, self.gh.EType.CONTAIN)

                    if arg_cnt == 4:
                        f_cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        f_cb_func = cwu.get_callback_function(f_cb_token)
                        if f_cb_func:
                            f_cb_id = self.gh.add_graph_node(f_cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                            self.gh.track_callback_function(f_cb_func, f_cb_id)
                            if cbg_id:
                                self.gh.add_graph_edge(cbg_id, f_cb_id, self.gh.EType.CONTAIN)


    ''' returns action server
        auto_start SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
        SimpleActionServer(name,execute_callback,auto_start)
        SimpleActionServer(name,auto_start)
        SimpleActionServer(nodehandle,name,execute_callback,auto_start)
        SimpleActionServer(nodehandle,name,auto_start)
        SimpleActionServer(name,execute_callback=NULL) DEPRECATED auto_start=true
        SimpleActionServer(nodehandle,name,execute_callback=NULL) DEPRECATED auto_start=true
        '''
    ##TODO handle auto_start?
    def process_simple_action_server(self, token, left_token, right_token):
        if token.str == 'SimpleActionServer' and cwu.is_match_previous(token, ['actionlib', '::']) and cwu.is_match(token.previous.astParent, '('):
            paren_token = token.previous.astParent
            serv_token = token.previous.astOperand2

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            new_thread = False
            node_token = None

            # arg_cnt == 1: new_thread = False, node_token = None
            if arg_cnt >= 2:
                first_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                (first_var, first_str) = cwu.get_var_token_and_var_name(first_token)
                if first_var and first_var.variable and cwu.find_sanitized_variable_type(first_var.variable).endswith('NodeHandle'):
                    node_token = first_token

                if arg_cnt == 2:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        new_thread = True
                elif arg_cnt == 3:
                    if node_token:
                        cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    else:
                        cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        new_thread = True
                elif arg_cnt == 4:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 2)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        new_thread = True

            if node_token:
                node_id = self.get_node_val_id(node_token)
                cbg_id = self.get_node_callback_group(node_token)
            else: # NODE_TOKEN = NULL
                if not self.current_node_id:
                    self.current_node_id = self.gh.setup_and_get_node()
                node_id = self.current_node_id
                cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)

            cb_id = None
            if new_thread: # cb_func is not None
                cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SERV})
                self.gh.track_callback_function(cb_func, cb_id)
                self.gh.add_graph_edge(node_id, cb_id, self.gh.EType.CONTROL) #NEW THREAD WITH INFINITE POLLING LOOP

            (serv_var, serv_str) = cwu.get_var_token_and_var_name(serv_token)
            if serv_var and serv_var.variable:
                serv_var.variable.val = cwu.update_val_dict(serv_var.variable.val, 'id', cbg_id)
                serv_var.variable.val = cwu.update_val_dict(serv_var.variable.val, 'e_cb_id', cb_id)
            
            paren_token.val = cwu.update_val_dict(paren_token.val, 'id', cbg_id)
            paren_token.val = cwu.update_val_dict(paren_token.val, 'e_cb_id', cb_id)
            ###TODO set id as cb_id and cbg as cbg_id
                    

    ''' simpleactionserver.registerGoalCallback(cb)
        '''
    def process_register_goal_callback(self, token, left_token, right_token):
        if token.str == 'registerGoalCallback' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            serv_token = token.astParent.astOperand1
            (serv_var, serv_str) = cwu.get_var_token_and_var_name(serv_token)
            if serv_var and serv_var.variable and serv_var.variable.val:
                cbg_id = serv_var.variable.val.get('id')
                e_cb_id = serv_var.variable.val.get('e_cb_id')

                arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                if arg_cnt == 1:
                    cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cb_func = cwu.get_callback_function(cb_token)
                    if cb_func:
                        # DONT ADD GOAL CALLBACK IF THERE IS EXECUTE CALLBACK
                        add = (e_cb_id == None)
                        cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB, 'add': add})
                        self.gh.track_callback_function(cb_func, cb_id)
                
                        if cbg_id and cb_id:
                            self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


    ''' simpleactionserver.registerPreemptCallback(cb)
        '''
    def process_register_result_callback(self, token, left_token, right_token):
        if token.str == 'registerPreemptCallback' and cwu.is_match(token.astParent, '.') and cwu.is_match(token.astParent.astParent, '('):
            paren_token = token.astParent.astParent
            serv_token = token.astParent.astOperand1
            cbg_id = cwu.get_val_id(serv_token)

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            if arg_cnt == 1:
                cb_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                cb_func = cwu.get_callback_function(cb_token)
                if cb_func:
                    cb_id = self.gh.add_graph_node(cb_func[1], self.gh.NType.CALLBACK_FN, {'cb_type': self.gh.CbFunc.ACT_SUB})
                    self.gh.track_callback_function(cb_func, cb_id)

                    if cbg_id and cb_id:
                        self.gh.add_graph_edge(cbg_id, cb_id, self.gh.EType.CONTAIN)


    ''' ros::MultiThreadedSpinner(thread_count=0)
        ros::AsyncSpinner(thread_count)
        ros::AsyncSpinner(thread_count,queue)
        '''
    def process_spinner(self, token, left_token, right_token):
        if token.str in ['MultiThreadedSpinner', 'AsyncSpinner'] and cwu.is_match_previous(token, ['ros','::']) and \
                cwu.is_match(token.previous.astParent, '('):
            paren_token = token.previous.astParent
            spin_token = token.previous.astOperand2

            arg_cnt = cwu.get_count_of_function_arguments(paren_token)
            thr_cnt = 1 # DEFAULT, because the value is changed to 1 if the cores value is 0.

            if arg_cnt == 0:
                thr_cnt = self.gh.ThrCount.CORES
            elif arg_cnt in [1, 2]:
                thr_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
                thr_cnt = cwu.get_numeric_value(thr_token)
                if thr_cnt == 0:
                    thr_cnt = self.gh.ThrCount.CORES

            if not self.current_node_id:
                self.current_node_id = self.gh.setup_and_get_node()
            node_id = self.current_node_id

            # UNLIKE ROS2, ALL NEW THREADS FOR MULTI AND ASYNC SPINNER
            thr_id = self.gh.setup_thread(thr_cnt) 

            # SPINNER IN ROS1 equivalent to EXECUTOR IN ROS2 with different scheduling scheme
            exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': thr_cnt, 'thr_id': thr_id})
            self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

            (spin_var, spin_str) = cwu.get_var_token_and_var_name(spin_token)
            if spin_var and spin_var.variable:
                spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'id', exec_id)
                spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'thr_count', thr_cnt)
                spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'thr_id', thr_id)

            paren_token.val = cwu.update_val_dict(paren_token.val, 'id', exec_id)
            paren_token.val = cwu.update_val_dict(paren_token.val, 'thr_count', thr_cnt)
            paren_token.val = cwu.update_val_dict(paren_token.val, 'thr_id', thr_id)

            # QUEUE IS ASSOCIATED WITH ASYNC SPINNER AT THE DECLARATION, WHEREAS 
            # QUEUE IS ASSOCIATED WITH SINGLE AND MULTI THREADED SPINNER AT SPIN
            if token.str == 'AsyncSpinner':
                cbg_id = None
                if arg_cnt == 2:
                    cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    cbg_id = cwu.get_val_id(cbg_token)
                
                if not cbg_id: # arg_cnt = 1 or cbg_token value is null
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)

                self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)


    ''' ros::spin() ==> global
        ros::spin(spinner) ==> global
        SingleThreadedSpinner.spin(cbg=0) ==> default: global
        MultiThreadedSpinner.spin(cbg=0) ==> default: global
        '''
    def process_spin(self, token, left_token, right_token):
        if token.str == 'spin' and cwu.is_match(token.next, '('):
            paren_token = token.next
            exec_id = None

            if cwu.is_match_previous(token, ['ros', '::']):
                if not self.current_node_id:
                    self.current_node_id = self.gh.setup_and_get_node()
                node_id = self.current_node_id

                arg_cnt = cwu.get_count_of_function_arguments(paren_token)

                if arg_cnt == 0:
                    # INTERNAL SINGLE THREADED SPINNER
                    exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
                    self.gh.update_node_to_implicit_executor(node_id, exec_id, setup=False)
                elif arg_cnt == 1:
                    spin_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                    exec_id = self.get_spinner_val_id(spin_token)

                if exec_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)
                    self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)

            elif cwu.is_match(token.astParent, '.'):
                spin_token = token.astParent.astOperand1
                exec_id = self.get_spinner_val_id(spin_token)

                if exec_id:
                    arg_cnt = cwu.get_count_of_function_arguments(paren_token)
                    cbg_id = None
                    if arg_cnt == 1:
                        cbg_token = cwu.get_comma_separated_token_at_pose_from_right(paren_token, 1)
                        cbg_id = cwu.get_val_id(cbg_token)

                    if not cbg_id:
                        if not self.current_node_id:
                            self.current_node_id = self.gh.setup_and_get_node()
                        node_id = self.current_node_id

                        cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)

                    self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

                    spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                    self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                    token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                    self.track_thr_token(token)

                    self.track_spin_locations(token)
                

    ''' AsyncSpinner.start()
        ##TODO support async spinner stop? 	
        '''
    def process_async_spinner_start(self, token, left_token, right_token):
        if token.str == 'start' and cwu.is_match(token.next, '(') and cwu.is_match(token.astParent, '.'):
            spin_token = token.astParent.astOperand1
            exec_id = self.get_spinner_val_id(spin_token)

            if exec_id:
                spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.INFINITE})
                self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

                token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
                self.track_thr_token(token) 

                self.track_spin_locations(token)  


    ''' ros::spinOnce() ==> global
        '''
    def process_spin_once(self, token, left_token, right_token):
        if token.str == 'spinOnce' and cwu.is_match(token.next, '(') and cwu.is_match_previous(token, ['ros', '::']):
            if not self.current_node_id:
                self.current_node_id = self.gh.setup_and_get_node()
            node_id = self.current_node_id

            # IMPLICIT EXECUTOR TO MODEL CALLAVAILABLE CALL ON GLOBAL QUEUE
            exec_id = self.gh.add_graph_node('default_exec', self.gh.NType.EXECUTOR, {'thr_count': 1})
            self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)
            self.gh.update_node_to_implicit_executor(node_id, exec_id, setup=False)

            cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)
            self.gh.add_graph_edge(exec_id, cbg_id, self.gh.EType.CONTAIN)

            spin_id = self.gh.add_graph_node('spin', self.gh.NType.SPIN, {'spin_type': self.gh.SpinType.AVAILABLE, 'time': 0})
            self.gh.add_graph_edge(spin_id, exec_id, self.gh.EType.INVOKE)

            token.val = cwu.update_val_dict(token.val, 'id', (spin_id, exec_id))
            self.track_thr_token(token)

            self.track_spin_locations(token)
 

    ##TODO CallbackQueue.callAvailable(), CallbackQueue.callAvailable(timeout)
    ##TODO CallbackQueue.callOne(), CallbackQueue.callOne(timeout)
            

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
                    cbg_id = self.get_node_callback_group(node_token)

                if cbg_id:
                    if sub_var and sub_var.variable:
                        sub_var.variable.val = cwu.update_val_dict(sub_var.variable.val, 'cbg', cbg_id)
                    else:
                        paren_token.val = cwu.update_val_dict(paren_token.val, 'cbg', cbg_id)


        elif token.previous and token.previous.str == '>' and token.previous.link and \
                token.previous.link.previous and token.previous.link.previous.str == 'Synchronizer':
            paren_token = None

            sync_token = token
            (sync_var, sync_str) = cwu.get_var_token_and_var_name(sync_token)
            if sync_var and sync_var.variable:
                if sync_token.astParent and sync_token.astParent.str == '(':
                    paren_token = sync_token.astParent
            elif sync_token.str == '(':
                paren_token = sync_token

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
                        if sync_var and sync_var.variable:
                            sync_var.variable.val = cwu.update_val_dict(sync_var.variable.val, 'cbg_set', cbg_set)
                        else:
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
            if cwu.is_match_previous(test_token, ['ros', '::', 'ok']):
                self.track_spin_locations(token)
        
                
    def process_main(self, function_object):
        if function_object and function_object.name == 'main':
            thr_func = (function_object, function_object.name)
            thr_id = self.gh.setup_thread(1)
            self.gh.track_thread_function(thr_func, thr_id)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # HELPER FUNCTIONS
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def get_spinner_val_id(self, spin_token):
        exec_id = ''
        (spin_var, spin_str) = cwu.get_var_token_and_var_name(spin_token)
        if spin_var and spin_var.variable and spin_var.variable.val:
            exec_id = spin_var.variable.val.get('id', '')
        elif spin_token.val:
            exec_id = spin_token.val.get('id', '')

        if not exec_id:
            # IF THERE IS NO AST FOR VARIABLE DECLARATION, EXECUTOR GRAPH NODE IS NOT THERE
            if spin_var and spin_var.variable:
                thr_cnt = None
                thr_id = None

                spinner_type = cwu.find_sanitized_variable_type(spin_var.variable)
                if spinner_type.endswith('MultiThreadedSpinner'):
                    thr_cnt = self.gh.ThrCount.CORES
                    thr_id = self.gh.setup_thread(thr_cnt)
                elif spinner_type.endswith('SingleThreadedSpinner'):
                    thr_cnt = 1
                    thr_id = self.gh.setup_thread(thr_cnt, setup_last_thr_id=False)
                elif spinner_type.endswith('AsyncSpinner'):
                    thr_cnt = self.gh.ThrCount.UNKNOWN
                    thr_id = self.gh.setup_thread(thr_cnt)

                if thr_cnt:
                    # EXECUTOR GRAPH NODE HAS NOT BEEN ADDED TO THE GRAPH; ADD ONE
                    exec_id = self.gh.add_graph_node('exec', self.gh.NType.EXECUTOR, {'thr_count': thr_cnt})
                    spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'id', exec_id)
                    spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'thr_count', thr_cnt)
                    spin_var.variable.val = cwu.update_val_dict(spin_var.variable.val, 'thr_id', thr_id)

                    # UNLIKE ROS2, SPINNER DECLARED IN ROS1 BELONGS TO THE CURRENT NODE
                    if not self.current_node_id:
                        self.current_node_id = self.gh.setup_and_get_node()
                    node_id = self.current_node_id
                    self.gh.add_graph_edge(node_id, exec_id, self.gh.EType.CONTAIN)

        return exec_id
    

    def update_and_get_node_id(self, node_variable):
        node_id = node_variable.val.get('id') if node_variable.val else None
        if node_id:
            if self.current_node_id and (node_id != self.current_node_id):
                # CURRENT NODE ID IS SET IN ROS::INIT, HENCE NOT EQUAL
                node_variable.val = cwu.update_val_dict(node_variable.val, 'id', self.current_node_id)
                node_id = self.current_node_id
        else:
            if not self.current_node_id:
                self.current_node_id = self.gh.setup_and_get_node()
            node_variable.val = cwu.update_val_dict(node_variable.val, 'id', self.current_node_id)
            node_id = self.current_node_id

        return node_id
    

    def get_node_val_id(self, node_token):
        node_id = ''
        (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
        if node_var and node_var.variable:
            node_id = self.update_and_get_node_id(node_var.variable)        
        return node_id


    def get_node_callback_group(self, node_token):
        cbg_id = None
        (node_var, node_str) = cwu.get_var_token_and_var_name(node_token)
        if node_var and node_var.variable:
            # ABSENCE OF 'cbg' INDICATES GLOBAL CALLBACK QUEUE
            cbg_id = node_var.variable.val.get('cbg') if node_var.variable.val else None
            if not cbg_id:
                node_id = self.update_and_get_node_id(node_var.variable)
                if node_id:
                    cbg_id = self.gh.get_default_cbgroup_of_node(node_id, setup=False)
        return cbg_id
    

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

