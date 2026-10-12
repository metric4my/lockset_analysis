#Copyright 2025 Purdue University.
#All rights reserved.


from graph_helper import GraphHelper
from error import AsyncError
from error_types import AsyncErrorTypes


class GraphChecker:

    def __init__(self, gh) -> None:
        self.gh = gh #GraphHelper()
        self.errors = []
        self.error_count = 0


    def add_error(self, e_type, sub_type, severity, values, ordered=True):
        found = False
        for e in self.errors:
            if ordered and e.ERROR_TYPE == e_type and e.sub_type == sub_type and e.values == values:
                found = True
            if (not ordered) and e.ERROR_TYPE == e_type and e.sub_type == sub_type and sorted(e.values) == sorted(values):
                found = True
            
        if not found:
            new_error = AsyncError()
            new_error.ERROR_TYPE = e_type
            new_error.sub_type = sub_type
            new_error.SEVERITY = severity
            new_error.values = values
            self.errors.append(new_error)
            self.error_count += 1


    def check(self):
        self.check_for_deadlock_in_callbacks()
        self.check_for_multiple_spins()
        self.check_for_multiple_callback_groups_with_single_thread()
        self.check_for_reentrant_callback_group_with_single_thread()
        self.check_for_spin_from_callbacks()
        self.check_for_nodes_improper_spins()
        self.check_for_disabled_callbacks()


    def check_for_deadlock_in_callbacks(self):
        cb_pairs = self.gh.get_invoke_callback_pairs()
        for (cb_from, cb_to) in cb_pairs:
            cbg_from = self.gh.get_callback_group_of_callback(cb_from)
            cbg_to = self.gh.get_callback_group_of_callback(cb_to)
            is_deadlock = False
            
            for cbg_u in cbg_from:
                for cbg_v in cbg_to:
                    if (cbg_u == cbg_v) and (self.gh.get_attrib_val(cbg_u, 'group_type') != self.gh.CbGroup.REENTRANT):
                        is_deadlock = True
                        break
                    else:
                        exec_from = self.gh.get_executor_of_callback_group(cbg_u)
                        exec_to = self.gh.get_executor_of_callback_group(cbg_v)
                        for exec_u in exec_from:
                            for exec_v in exec_to:
                                if (exec_u == exec_v) and self.gh.get_attrib_val(exec_u, 'thr_count') == 1:
                                    is_deadlock = True
                                    for spin_id in self.gh.get_spin_invokes_of_executor(exec_u):
                                        if self.gh.has_edge(cb_from, spin_id):
                                            is_deadlock = False
                                            break
                                    if not is_deadlock:
                                        break
                            if not is_deadlock:
                                break
                        if is_deadlock:
                            break
                if is_deadlock:
                    break
                            
            if is_deadlock:
                self.add_error(AsyncErrorTypes.DEADLOCK_IN_CALLBACKS, '', AsyncErrorTypes.SEVERITY.ERROR, (cb_from, cb_to))

                
    def check_for_multiple_spins(self):
        exec_list = self.gh.get_executors_all()
        for exec_id in exec_list:
            spin_list = self.gh.get_spin_invokes_of_executor(exec_id)
            # CHECK IF SPINS ARE IN MULTIPLE THREADS
            thr_id = None
            for spin in spin_list:
                if not thr_id:
                    thr_id = self.gh.get_attrib_val(spin, 'thr_id')
                else:
                    thr2_id = self.gh.get_attrib_val(spin, 'thr_id') 
                    if thr2_id and thr2_id != thr_id:
                        self.add_error(AsyncErrorTypes.SPIN_FROM_MULTIPLE_THREADS, '', AsyncErrorTypes.SEVERITY.ERROR, exec_id)
                        break

        # CHECK FOR MULTIPLE BLOCKING SPINS
        # all_spin_list = self.gh.get_spins_all()

        all_spin_list = []
        exec_list = self.gh.get_executors_all()
        for exec_id in exec_list:
            spin_list = self.gh.get_spin_invokes_of_executor(exec_id)
            if spin_list:
                all_spin_list.extend(spin_list)
        infinite_spin_list = [spin_id for spin_id in all_spin_list if self.gh.get_attrib_val(spin_id, 'spin_type')==self.gh.SpinType.INFINITE]

        # thr_id = None
        # for spin_id in infinite_spin_list:
        #     if self.gh.get_attrib_val(spin_id, 'spin_type')==self.gh.SpinType.INFINITE:
        #         if not thr_id:
        #             thr_id = self.gh.get_attrib_val(spin_id, 'thr_id')
        #         else:
        #             thr2_id = self.gh.get_attrib_val(spin_id, 'thr_id')
        #             if thr2_id and thr2_id == thr_id:
        #                 self.add_error(AsyncErrorTypes.MULTIPLE_BLOCKING_SPINS, '', AsyncErrorTypes.SEVERITY.ERROR, None)
        #                 break

        thr_id = None
        multi_block_thrs = []
        for i in range(len(infinite_spin_list)):
            thr_id = self.gh.get_attrib_val(infinite_spin_list[i], 'thr_id')
            if thr_id and (thr_id not in multi_block_thrs):
                for j in range(i+1, len(infinite_spin_list)):
                    thr2_id = self.gh.get_attrib_val(infinite_spin_list[j], 'thr_id')
                    if thr2_id and thr2_id == thr_id:
                        multi_block_thrs.append(thr_id)
                        self.add_error(AsyncErrorTypes.MULTIPLE_BLOCKING_SPINS, '', AsyncErrorTypes.SEVERITY.ERROR, thr_id)
                        break
            

    ##TODO reformat to check for mulitple callback groups of a single node with single thread? 
    ##If so, use get callback groups of a node and get executors of a node for each node
    def check_for_multiple_callback_groups_with_single_thread(self):
        cbg_list = self.gh.get_callback_groups_all()
        exec_list = self.gh.get_executors_all()
        for exec_id in exec_list:
            if self.gh.get_attrib_val(exec_id, 'thr_count') == 1:
                first_cbg = None
                for cbg_id in cbg_list:
                    if not self.gh.has_outgoing_edge(cbg_id):
                        # no callbacks associated to this group, skip it
                        continue
                    if exec_id in self.gh.get_executor_of_callback_group(cbg_id):
                        if first_cbg:
                            self.add_error(AsyncErrorTypes.SINGLE_THREAD_FOR_PARALLEL_CALLBACKS, 'MULTIPLE', AsyncErrorTypes.SEVERITY.WARN, (exec_id, first_cbg, cbg_id))
                            break
                        else:
                            first_cbg = cbg_id
    

    def check_for_reentrant_callback_group_with_single_thread(self):
        cbg_list = self.gh.get_callback_groups_all()
        for cbg_id in cbg_list:
            # AVOID ERROS FOR THE INTERNAL RATE CALLBACK GROUP OF RCLPY NODE
            if cbg_id.startswith('rate_cbg_'):
                continue
            if self.gh.get_attrib_val(cbg_id, 'group_type') == self.gh.CbGroup.REENTRANT:
                exec_list = self.gh.get_executor_of_callback_group(cbg_id)
                for exec_id in exec_list:
                    if self.gh.get_attrib_val(exec_id, 'thr_count') == 1:
                        self.add_error(AsyncErrorTypes.SINGLE_THREAD_FOR_PARALLEL_CALLBACKS, 'REENTRANT', AsyncErrorTypes.SEVERITY.WARN, (cbg_id, exec_id))


    def check_for_spin_from_callbacks(self):
        cycles_list =  self.gh.get_cycles()

        for cycle in cycles_list:
            cycle_spins = []
            # has_spin = False
            cb_id = None
            for gr_node in cycle:
                node_type = self.gh.get_attrib_val(gr_node, 'type') 
                if node_type == self.gh.NType.SPIN:
                    # has_spin = True
                    cycle_spins.append(gr_node)
                elif node_type == self.gh.NType.CALLBACK_FN:
                    cb_id = gr_node
            # If cycle contains spin node, there is an edge from callback to spin
            # if has_spin and cb_id:
            #     self.add_error(AsyncErrorTypes.SPIN_FROM_CALLBACK, 'OTHER', AsyncErrorTypes.SEVERITY.WARN, cb_id)

            ##TODO more checks; and may be remove the above error check after more checks are added?
            if cb_id:
                for spin_id in cycle_spins:
                    sub_type = 'BLOCKING' if self.gh.get_attrib_val(spin_id, 'spin_type') == self.gh.SpinType.INFINITE else 'TIMED'
                    level = AsyncErrorTypes.SEVERITY.ERROR if sub_type == 'BLOCKING' else AsyncErrorTypes.SEVERITY.WARN
                    self.add_error(AsyncErrorTypes.SPIN_FROM_CALLBACK, sub_type, level, (cb_id, spin_id))

            # there is no incoming invocation edge to spin node, 
            # which means there is no other spin call that can invoke the callback function of this spin node
            if len(cycle_spins) == 1: 
                if cb_id:
                    node_list = self.gh.get_node_of_callback(cb_id)
                    # a callback can belong to only one node; so node_list should have only one entry
                    if node_list and self.gh.get_attrib_val(node_list[0], 'is_active') != True:
                        return
                spin_list = self.gh.get_spin_invokes_of_spin(cycle_spins[0])
                no_invoke = False
                for spin_id in spin_list:
                    if not self.gh.has_incoming_edge(spin_id):
                        no_invoke = True
                        break
                if not no_invoke:
                    self.add_error(AsyncErrorTypes.MISSING_SPIN, 'MISSING', AsyncErrorTypes.SEVERITY.ERROR, (cb_id, cycle_spins[0]))


    def check_for_nodes_improper_spins(self):
        nodes_list = self.gh.get_ros_nodes_all()
        for node_id in nodes_list:
            spin_num = 0
            node_spin_list = []
            exec_list = self.gh.get_executor_of_node(node_id)
            for exec_id in exec_list:
                spin_list = self.gh.get_spin_invokes_of_executor(exec_id)
                # filter out the dummy spins
                spin_list = [spin_id for spin_id in spin_list if self.gh.get_attrib_val(spin_id, 'dummy')!=True]
                spin_num += len(spin_list)
                node_spin_list.extend(spin_list)

            # CHECK FOR MISSING SPIN
            if self.gh.get_attrib_val(node_id, 'is_active')==True and spin_num == 0:
                # CHECK IF NODE HAS CALLBACKS
                if self.has_callbacks_for_node(node_id):
                    self.add_error(AsyncErrorTypes.MISSING_SPIN, 'NONE', AsyncErrorTypes.SEVERITY.ERROR, node_id)
            # elif spin_num > 1:
            #     print("Error#: Node %s is having %d spin calls" % (node, spin_num))

            elif node_spin_list:
                # CHECK FOR VERSION SPECIFIC SPINS
                spin_edges = self.gh.get_invoke_spin_pairs_from_spin_list(node_spin_list)
                ver = self.gh.get_attrib_val(node_id, 'ver')
                if ver == self.gh.Version.ROSCPP:
                    self.check_for_roscpp_spins(spin_edges, node_id)
                elif ver == self.gh.Version.ROSPY:
                    self.check_for_rospy_spins(spin_edges, node_id)
                elif ver == self.gh.Version.RCLCPP:
                    self.check_for_rclcpp_spins(spin_edges, node_id)
                elif ver == self.gh.Version.RCLPY:
                    self.check_for_rclpy_spins(spin_edges, node_id)

                # CHECK FOR MULTIPLE BLOCKING SPINS
                # infinite_spin_list = [spin_id for spin_id in node_spin_list if self.gh.get_attrib_val(spin_id, 'spin_type')==self.gh.SpinType.INFINITE]
                # thr_id = None
                # for spin_id in infinite_spin_list:
                #     if self.gh.get_attrib_val(spin_id, 'spin_type')==self.gh.SpinType.INFINITE:
                #         if not thr_id:
                #             thr_id = self.gh.get_attrib_val(spin_id, 'thr_id')
                #         elif self.gh.get_attrib_val(spin_id, 'thr_id') == thr_id:
                #             self.add_error(AsyncErrorTypes.MULTIPLE_BLOCKING_SPINS, '', AsyncErrorTypes.SEVERITY.ERROR, node_id)
                #             break

    
    def check_for_disabled_callbacks(self):
        gr_node_list = self.gh.get_graph_nodes_with_local_scope()
        for gr_node in gr_node_list:
            node_type = self.gh.get_attrib_val(gr_node, 'type')
            if node_type == self.gh.NType.NODE:
                pass # DISABED THE CHECK FOR NODE
                # # CHECK IF NODE HAS CALLBACKS
                # if self.has_callbacks_for_node(gr_node):
                #     self.add_error(AsyncErrorTypes.DISABLED_CALLBACK, 'NODE', AsyncErrorTypes.SEVERITY.ERROR, gr_node)
            elif node_type == self.gh.NType.CALLBACK_GROUP:
                self.add_error(AsyncErrorTypes.DISABLED_CALLBACK, 'GROUP', AsyncErrorTypes.SEVERITY.ERROR, gr_node)
            elif node_type == self.gh.NType.CALLBACK_FN:
                self.add_error(AsyncErrorTypes.DISABLED_CALLBACK, 'CALLBACK', AsyncErrorTypes.SEVERITY.ERROR, gr_node)


    def check_for_roscpp_spins(self, spin_pairs, node_id):
        # multiple executors of node that are of different thread types not supported
        for (u, v) in spin_pairs:
            exec_u = self.gh.get_executor_of_spin(u)
            exec_v = self.gh.get_executor_of_spin(v)

            if len(exec_u) != 1 or len(exec_v) != 1:
                # spin can be associated with only one executor
                return
            
            exec_u = exec_u[0]
            exec_v = exec_v[0]
            
            if exec_u != exec_v and self.gh.get_attrib_val(exec_u, 'thr_count') != self.gh.get_attrib_val(exec_v, 'thr_count'):
                self.add_error(AsyncErrorTypes.MULTIPLE_EXECUTORS, 'DIFFERENT', AsyncErrorTypes.SEVERITY.ERROR, (exec_u, exec_v, node_id), ordered=False)


    def check_for_rospy_spins(self, spin_pairs, node_id):
        pass


    def check_for_rclcpp_spins(self, spin_pairs, node_id):
        for (u, v) in spin_pairs:
            exec_u = self.gh.get_executor_of_spin(u)
            exec_v = self.gh.get_executor_of_spin(v)

            if len(exec_u) != 1 or len(exec_v) != 1:
                # spin can be associated with only one executor
                return
            
            exec_u = exec_u[0]
            exec_v = exec_v[0]

            # nested spin calls not supported (same and/or different executors)
            if exec_u == exec_v:
                self.add_error(AsyncErrorTypes.NESTED_SPINS, 'ANY', AsyncErrorTypes.SEVERITY.ERROR, (u, v, exec_u))

            # multiple executors of node (via add_node) not supported
            else: # exec_u != exec_v
                self.add_error(AsyncErrorTypes.MULTIPLE_EXECUTORS, 'ANY', AsyncErrorTypes.SEVERITY.ERROR, (exec_u, exec_v, node_id), ordered=False)


    def check_for_rclpy_spins(self, spin_pairs, node_id):
        for (u, v) in spin_pairs:
            exec_u = self.gh.get_executor_of_spin(u)
            exec_v = self.gh.get_executor_of_spin(v)

            if len(exec_u) != 1 or len(exec_v) != 1:
                # spin can be associated with only one executor
                return
            
            exec_u = exec_u[0]
            exec_v = exec_v[0]

            # multiple executors of node not supported, the latter executor disables the others by removing the node from them
            if exec_u != exec_v:
                self.add_error(AsyncErrorTypes.MULTIPLE_EXECUTORS, 'ANY', AsyncErrorTypes.SEVERITY.ERROR, (exec_u, exec_v, node_id), ordered=False)

            # nested spin calls with blocking spin as the outer one (same executor) not supported
            # Note: if there is edge from v to u, then both u and v are invoked from callbacks
            elif self.gh.get_attrib_val(u, 'spin_type') == self.gh.SpinType.INFINITE and (not self.gh.has_edge(v, u)):
                # check which nodes the spins are called on. rclpy spin APIs take node as an argument.
                # if nodes are same, then inner one will remove the node from the executor disabling the outer spin.
                if self.gh.get_attrib_val(u, 'node') == self.gh.get_attrib_val(v, 'node'):
                    self.add_error(AsyncErrorTypes.NESTED_SPINS, 'BLOCKING', AsyncErrorTypes.SEVERITY.ERROR, (u, v, exec_u))


    # Helper function: CHECK IF NODE HAS CALLBACKS
    def has_callbacks_for_node(self, node_id):
        has_cb = False
        cbg_list = self.gh.get_callback_groups_of_node(node_id)
        for cbg_id in cbg_list:
            cb_list = self.gh.get_callbacks_of_callback_group(cbg_id)
            if cb_list:
                has_cb = True
                break
        return has_cb


    def print_errors(self):
        print('- '*40)
        print("Total count of errors and warnings: %s" % self.error_count)
        self.pretty_print()
        print('- '*40)


    def pretty_print(self, show_errors=True, show_warnings=True):
        ''' PRINTS ERRORS TO STD OUT, ATTEMPTS TO BE HUMAN READABLE
            '''

        for e in self.errors:

            is_error = (e.SEVERITY == AsyncErrorTypes.SEVERITY.ERROR)
            if is_error and not show_errors:
                continue

            is_warning = (e.SEVERITY == AsyncErrorTypes.SEVERITY.WARN)
            if is_warning and not show_warnings:
                continue

            print('- '*40)

            if e.ERROR_TYPE == AsyncErrorTypes.DEADLOCK_IN_CALLBACKS:
                print(e.get_error_desc())
                cb_from, cb_to = e.values
                print("Error#: potential deadlock: %s -- %s" % (cb_from, cb_to))

            elif e.ERROR_TYPE == AsyncErrorTypes.SPIN_FROM_MULTIPLE_THREADS:
                print(e.get_error_desc())
                exec_id = e.values
                print("Error#: Spin of executor %s in multiple threads" % exec_id)

            elif e.ERROR_TYPE == AsyncErrorTypes.SPIN_FROM_CALLBACK:
                print(e.get_error_desc())
                if e.sub_type == 'BLOCKING':
                    cb_id, spin_id = e.values
                    print("Error#: blocking spin %s is invoked from a callback function %s" % (spin_id, cb_id))
                elif e.sub_type == 'TIMED':
                    cb_id, spin_id = e.values
                    print("Warning#: spin %s is invoked from a callback function %s" % (spin_id, cb_id))
                elif e.sub_type == 'OTHER':
                    cb_id = e.values
                    print("Error#: Spin is invoked from a callback function %s" % cb_id)

            elif e.ERROR_TYPE == AsyncErrorTypes.MISSING_SPIN:
                print(e.get_error_desc())
                if e.sub_type == 'MISSING':
                    cb_id, spin_id = e.values
                    print("Error#: Missing spin call that can execute the callback function %s for the first time which in turn can invoke spin %s" \
                            % (cb_id, spin_id))
                elif e.sub_type == 'NONE':
                    node_id = e.values
                    print("Error#: Node %s is missing a spin call" % node_id)

            elif e.ERROR_TYPE == AsyncErrorTypes.MULTIPLE_BLOCKING_SPINS:
                print(e.get_error_desc())
                # node_id = e.values
                # print("Error#: Node %s has multiple blocking spins from the same thread. The program will block at the first one" % node_id)
                thr_id = e.values
                print("Error#: There are multiple blocking spins from the same thread (id: %s). The program will block at the first one" % thr_id)

            elif e.ERROR_TYPE == AsyncErrorTypes.NESTED_SPINS:
                print(e.get_error_desc())
                (outer_spin_id, inner_spin_id, exec_id) = e.values
                if e.sub_type == 'ANY':
                    print("Error#: Spin %s is invoked when the executor %s is already spinning with %s" % (inner_spin_id, exec_id, outer_spin_id))
                elif e.sub_type == 'BLOCKING':
                    print("Error#: Spin %s is invoked when the executor %s is already spinning with blocking %s. Inner spin %s may potentially disable the outer spin %s" \
                            % (inner_spin_id, exec_id, outer_spin_id, inner_spin_id, outer_spin_id))

            elif e.ERROR_TYPE == AsyncErrorTypes.MULTIPLE_EXECUTORS:
                print(e.get_error_desc())
                (exec_1, exec_2, node_id) = e.values
                if e.sub_type == 'ANY':
                    print("Error#: Node %s is added to another executor %s when it has already been added to the executor %s. Node can have only one executor at a time." \
                            % (node_id, exec_2, exec_1))
                elif e.sub_type == 'DIFFERENT':
                    print("Error#: Node %s is added to multiple executors of different thread types: %s %s" % (node_id, exec_1, exec_2))
                    
            elif e.ERROR_TYPE == AsyncErrorTypes.SINGLE_THREAD_FOR_PARALLEL_CALLBACKS:
                print(e.get_error_desc())
                if e.sub_type == 'MULTIPLE':
                    exec_id, first_cbg, cbg_id = e.values
                    print("Warning#: Single threaded executor %s has more than one callback group: %s %s" % (exec_id, first_cbg, cbg_id))
                elif e.sub_type == 'REENTRANT':
                    cbg_id, exec_id = e.values
                    print("Warning#: Reentrant callback group %s is in a single threaded executor %s" % (cbg_id, exec_id))

            elif e.ERROR_TYPE == AsyncErrorTypes.DISABLED_CALLBACK:
                print(e.get_error_desc())
                gr_node = e.values
                if e.sub_type == 'NODE':
                    print("Error#: all the callbacks associated with node %s are disabled, as the node variable is declared locally" % gr_node)
                elif e.sub_type == 'GROUP':
                    print("Error#: all the callbacks associated with callback group %s are disabled, as the callback group variable is declared locally" % gr_node)
                elif e.sub_type == 'CALLBACK':
                    print("Error#: the callback %s is disabled, as the object variable associated with the callback is declared locally" % gr_node)


    def print_errors_to_file(self, errors_file='errors.txt', restart=False, show_errors=True, show_warnings=True):
        mode = 'w' if restart else 'a'

        with open(errors_file, mode) as f:

            for e in self.errors:
                is_error = (e.SEVERITY == AsyncErrorTypes.SEVERITY.ERROR)
                if is_error and not show_errors:
                    continue

                is_warning = (e.SEVERITY == AsyncErrorTypes.SEVERITY.WARN)
                if is_warning and not show_warnings:
                    continue

                etype = e.get_error_desc()

                if e.ERROR_TYPE == AsyncErrorTypes.DEADLOCK_IN_CALLBACKS:
                    cb_from, cb_to = e.values
                    f.write("\n%s; Error#: potential deadlock: %s -- %s" % (etype, cb_from, cb_to))

                elif e.ERROR_TYPE == AsyncErrorTypes.SPIN_FROM_MULTIPLE_THREADS:
                    exec_id = e.values
                    f.write("\n%s; Error#: Spin of executor %s in multiple threads" % (etype, exec_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.SPIN_FROM_CALLBACK:
                    if e.sub_type == 'BLOCKING':
                        cb_id, spin_id = e.values
                        f.write("\n%s; Error#: blocking spin %s is invoked from a callback function %s" % (etype, spin_id, cb_id))
                    elif e.sub_type == 'TIMED':
                        cb_id, spin_id = e.values
                        f.write("\n%s; Warning#: spin %s is invoked from a callback function %s" % (etype, spin_id, cb_id))
                    elif e.sub_type == 'OTHER':
                        cb_id = e.values
                        f.write("\n%s; Error#: Spin is invoked from a callback function %s" % (etype, cb_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.MISSING_SPIN:
                    if e.sub_type == 'MISSING':
                        cb_id, spin_id = e.values
                        f.write("\n%s; Error#: Missing spin call that can execute the callback function %s for the first time which in turn can invoke spin %s" \
                                % (etype, cb_id, spin_id))
                    elif e.sub_type == 'NONE':
                        node_id = e.values
                        f.write("\n%s; Error#: Node %s is missing a spin call" % (etype, node_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.MULTIPLE_BLOCKING_SPINS:
                    # node_id = e.values
                    # f.write("\n%s; Error#: Node %s has multiple blocking spins from the same thread. The program will block at the first one" % (etype, node_id))
                    thr_id = e.values
                    f.write("\n%s; Error#: There are multiple blocking spins from the same thread (id: %s). The program will block at the first one" % (etype, thr_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.NESTED_SPINS:
                    (outer_spin_id, inner_spin_id, exec_id) = e.values
                    if e.sub_type == 'ANY':
                        f.wrtie("\n%s; Error#: Spin %s is invoked when the executor %s is already spinning with %s" % (etype, inner_spin_id, exec_id, outer_spin_id))
                    elif e.sub_type == 'BLOCKING':
                        f.write("\n%s; Error#: Spin %s is invoked when the executor %s is already spinning with blocking %s. Inner spin %s may potentially disable the outer spin %s" \
                                % (etype, inner_spin_id, exec_id, outer_spin_id, inner_spin_id, outer_spin_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.MULTIPLE_EXECUTORS:
                    (exec_1, exec_2, node_id) = e.values
                    if e.sub_type == 'ANY':
                        f.write("\n%s; Error#: Node %s is added to another executor %s when it has already been added to the executor %s. Node can have only one executor at a time." \
                                % (etype, node_id, exec_2, exec_1))
                    elif e.sub_type == 'DIFFERENT':
                        f.write("\n%s; Error#: Node %s is added to multiple executors of different thread types: %s %s" % (etype, node_id, exec_1, exec_2))
                    
                elif e.ERROR_TYPE == AsyncErrorTypes.SINGLE_THREAD_FOR_PARALLEL_CALLBACKS:
                    if e.sub_type == 'MULTIPLE':
                        exec_id, first_cbg, cbg_id = e.values
                        f.write("\n%s; Warning#: Single threaded executor %s has more than one callback group: %s %s" % (etype, exec_id, first_cbg, cbg_id))
                    elif e.sub_type == 'REENTRANT':
                        cbg_id, exec_id = e.values
                        f.write("\n%s; Warning#: Reentrant callback group %s is in a single threaded executor %s" % (etype, cbg_id, exec_id))

                elif e.ERROR_TYPE == AsyncErrorTypes.DISABLED_CALLBACK:
                    gr_node = e.values
                    if e.sub_type == 'NODE':
                        f.write("\n%s; Error#: all the callbacks associated with node %s are disabled, as the node variable is declared locally" % (etype, gr_node))
                    elif e.sub_type == 'GROUP':
                        f.write("\n%s; Error#: all the callbacks associated with callback group %s are disabled, as the callback group variable is declared locally" % (etype, gr_node))
                    elif e.sub_type == 'CALLBACK':
                        f.write("\n%s; Error#: the callback %s is disabled, as the object variable associated with the callback is declared locally" % (etype, gr_node))




