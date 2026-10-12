#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


from graph_helper import GraphHelper
from cpp_walker import CPPWalker
from cpp1_walker import CPP1Walker
from py_walker import PYWalker
from py1_walker import PY1Walker
from py_walker_utils import PYSymbolHelper
from filegroups_manager import FileGroupsManager
from functions_manager import FunctionsManager
from scopes_manager import ScopesManager
from graph_checker import GraphChecker
import cppcheckdata 
import ast
import fnmatch
import os                



class GraphCollector:
    def __init__(self):
        self.analyze_single_file = False
        self.current_file_under_analysis = ''
        self.source_file = ''
        self.current_ros_version = None
        self.configurations = []
        self.all_sorted_analysis_dicts = []
        self.should_sort_by_function_graph = True
        self.cb_link_track_items = []
        # self.gh = GraphHelper()
        # self.fm = FunctionsManager()
        # self.sm = ScopesManager(self.gh)
        # self.psh = PYSymbolHelper(self.sm)
        ####TODO
        self.gh = None
        self.fm = None
        self.sm = None
        self.psh = None
        self.cw = None
        self.pw = None


    def init_objects(self):
        self.gh = GraphHelper()
        self.fm = FunctionsManager()
        self.sm = ScopesManager(self.gh)
        self.psh = PYSymbolHelper(self.sm)


    def construct_check_and_display_graph(self, target, launch_file, display_name='', errors_file=''):
        if os.path.isdir(target):

            fgm = FileGroupsManager()
            file_groups = fgm.get_file_groups(target, launch_file)

            for group_id, group_info in file_groups.items():
                #print("group id = %s\n ros_version: %s\n" % (group_id, group_info['ros_version']))
                ####TODO
                self.init_objects()

                if 'cpp_files' in group_info:
                    try:
                        ros_version = group_info['ros_version']
                        self.configure_collector('CPP', ros_version)

                        group = group_info['cpp_files']
                        group = set(group)
                        for group_file in group:
                            source_file = os.path.join(target, group_file)
                            dump_file = source_file + '.dump'
                            if not os.path.exists(dump_file):
                                continue
                            print("Found dump: %s" % os.path.relpath(dump_file, target))
                            self.construct_from_cpp(dump_file, source_file)
                    except Exception as e:
                        print("EXCEPTION: %s" % str(e))
                    else:
                        ####TODO uncomment
                        #self.cw.reset_current_node_id()
                        self.post_process_collect_from_file_group()
                    finally:
                        self.print_filegroup_to_file(group, errors_file)
                        self.unconfigure_collector()

                elif 'py_files' in group_info:
                    try:
                        ros_version = group_info['ros_version']
                        self.configure_collector('PY', ros_version)

                        group = group_info['py_files']
                        group = set(group)
                        for group_file in group:
                            source_file = os.path.join(target, group_file)
                            if not os.path.exists(source_file):
                                continue
                            print("Found script: %s" % os.path.relpath(source_file, target))
                            self.construct_from_py(source_file)                 
                    except Exception as e:
                        print("EXCEPTION: %s" % str(e))
                    else:
                        self.post_process_collect_from_file_group()
                    finally:
                        self.print_filegroup_to_file(group, errors_file)
                        self.unconfigure_collector()

                ####TODO
                self.post_process_collect()
                self.check_graph(errors_file)

                if display_name:
                    graph_name = display_name + '/graph_' + group_id.replace('.', '_').replace('/', '_') + '.png'
                    self.gh.print_graph(graph_name)

        else:
            source_file = target
            ####TODO
            self.init_objects()

            if fnmatch.fnmatch(target, "*.cpp"):
                self.analyze_single_file = True
                self.construct_from_cpp(target + '.dump', source_file)

                self.post_process_collect_from_file_group()
                self.post_process_collect()

            elif fnmatch.fnmatch(target, "*.py") or ('.' not in target):
                self.analyze_single_file = True
                self.construct_from_py(source_file)

                self.post_process_collect_from_file_group()
                self.post_process_collect()

            ####TODO
            self.check_graph(errors_file)

            if display_name:
                self.gh.print_graph(display_name)


    def check_graph(self, errors_file=''):
        gc = GraphChecker(self.gh)
        gc.check()
        gc.print_errors()
        if errors_file:
            gc.print_errors_to_file(errors_file)


    def construct_graph(self, target, launch_file):
        self.init_objects()

        if os.path.isdir(target):

            fgm = FileGroupsManager()
            file_groups = fgm.get_file_groups(target, launch_file)

            for group_id, group_info in file_groups.items():
                ####TODO
                # self.init_objects()

                if 'cpp_files' in group_info:
                    ros_version = group_info['ros_version']
                    self.configure_collector('CPP', ros_version)

                    group = group_info['cpp_files']
                    for group_file in group:
                        source_file = os.path.join(target, group_file)
                        dump_file = source_file + '.dump'
                        if not os.path.exists(dump_file):
                            continue
                        print("Found dump: %s" % os.path.relpath(dump_file, target))
                        self.construct_from_cpp(dump_file, source_file)

                    ####TODO uncomment
                    #self.cw.reset_current_node_id()
                    self.post_process_collect_from_file_group()
                    self.unconfigure_collector()

                elif 'py_files' in group_info:
                    ros_version = group_info['ros_version']
                    self.configure_collector('PY', ros_version)

                    group = group_info['py_files']
                    for group_file in group:
                        source_file = os.path.join(target, group_file)
                        if not os.path.exists(source_file):
                            continue
                        print("Found script: %s" % os.path.relpath(source_file, target))
                        self.construct_from_py(source_file)

                    self.post_process_collect_from_file_group()
                    self.unconfigure_collector()

            self.post_process_collect()

                ####TODO
                # self.post_process_collect()
                # self.check_graph()
        else:
            source_file = target
            ####TODO
            # self.init_objects()

            if fnmatch.fnmatch(target, "*.cpp"):
                self.analyze_single_file = True
                self.construct_from_cpp(target + '.dump', source_file)

                self.post_process_collect_from_file_group()
                self.post_process_collect()

            elif fnmatch.fnmatch(target, "*.py") or ('.' not in target):
                self.analyze_single_file = True
                self.construct_from_py(source_file)

                self.post_process_collect_from_file_group()
                self.post_process_collect()

            ####TODO
            # self.check_graph()


    def construct_from_cpp(self, dump_file, source_file=''):
        ''' input: a cppcheck 'dump' file containing an Abstract Syntax Tree (AST), symbol table, and token list
            input: a cpp source file
            returns: None
            side-effect: updates cppcheck data structure with information about this analysis
            '''
        self.current_file_under_analysis = dump_file
        self.source_file = source_file

        # PARSE FILE
        data = cppcheckdata.parsedump(dump_file)
        
        #TODO check for multiple? Now, ONLY TEST THE FIRST CONFIGURATION
        for c in data.configurations[:1]:
            c = self.init_cppcheck_config_data_structures(c)

            self.configure_collector('CPP', cppcheck_configuration=c)

            #TODO do we need it?
            #self.sh.fill_func_name_dict(c.functions)
 
            # GET DICT OF ALL GLOBALLY SCOPED FUNCTIONS          
            analysis_dict = self.fm.find_functions(c)
            # FIND ORDER FOR FUNCTION GRAPH EXPLORATION
            sorted_analysis_dict = analysis_dict
            if self.should_sort_by_function_graph:
                self.fm.build_function_graph(analysis_dict)
                sorted_analysis_dict = self.fm.make_sorted_analysis_dict_from_function_graph(analysis_dict)
            self.all_sorted_analysis_dicts.append(sorted_analysis_dict)

            #TODO do we need it?
            #self.collect_types_outside_functions(c.tokenlist)
            
            # RUN TYPE ANNOTATOR ON ALL TOKEN PARSE TREES FOR EACH FUNCTION
            for function_dict in sorted_analysis_dict.values():
                self.collect_from_cpp_function_dict(function_dict)

            self.post_process_collect_from_cpp(analysis_dict)

            #TODO do we need it?
            #self.sh.clear_func_name_dict()
            self.configurations.append(c)


    def init_cppcheck_config_data_structures(self, cppcheck_configuration):  
        ''' AUGMENT CPPCHECK DATA STRUCTURE WITH ADDITIONAL FEATURES TO SUPPORT THIS ANALYSIS
            '''
        c = cppcheck_configuration

        for t in c.tokenlist:
            t.isRoot = False
            ##TODO instead save token to val dictionary, as only few tokens may get a value? ##
            t.val = None 

        for v in c.variables:
            v.val = None

        return c
    

    def collect_from_cpp_function_dict(self, function_dict):
        for root_token in function_dict['root_tokens']:                 
            self.cw.collect(root_token)

        self.cw.collect_from_function(function_dict['function'])



    def construct_from_py(self, source_file):
        self.current_file_under_analysis = source_file
        self.source_file = source_file

        with open(source_file, 'r') as fp:
            code = fp.read()

            ##TODO annotate ast with symbols instead of looking up symbols for ast nodes?
            self.sm.find_scopes(code)
            self.sm.build_function_graph(source_file)
            tree = ast.parse(code)

            self.configure_collector('PY', pyast_tree=tree)

            self.pw.collect(tree)

            self.post_process_collect_from_py()


    def configure_collector(self, collect_type, ros_version=None, cppcheck_configuration=None, pyast_tree=None):
        if not self.current_ros_version:
            if ros_version:
                self.current_ros_version = ros_version
            elif cppcheck_configuration:
                ros_version = 1
                for directive in cppcheck_configuration.directives:
                    if (directive.str.startswith('#include')) and ('rclcpp.hpp' in directive.str[8:]):
                        ros_version = 2
            elif pyast_tree:
                iv = ImportVisitor()
                iv.visit(pyast_tree)
                ros_version = iv.get_ros_version()
                if not ros_version:
                    ros_version = 1
            
            if ros_version:
                if collect_type == 'CPP':
                    self.gh.set_current_version(self.gh.Version.ROSCPP) if ros_version == 1 else self.gh.set_current_version(self.gh.Version.RCLCPP)
                    self.cw = CPP1Walker(self.gh) if ros_version == 1 else CPPWalker(self.gh)
                elif collect_type == 'PY':
                    self.gh.set_current_version(self.gh.Version.ROSPY) if ros_version == 1 else self.gh.set_current_version(self.gh.Version.RCLPY)
                    self.pw = PY1Walker(self.gh, self.psh) if ros_version == 1 else PYWalker(self.gh, self.psh)


    def unconfigure_collector(self):
        self.current_ros_version = None


    # POST PROCESS CALLBACK  LINKS
    def post_process_collect_from_cpp(self, analysis_dict):
        cb_link_track_items = self.cw.get_cb_link_track_items()

        for (func, id_string) in cb_link_track_items:
            calling_func_list = self.fm.get_calling_functions_list(func, analysis_dict)
            self.cb_link_track_items.append((calling_func_list, id_string))

        self.cw.clear_cb_link_track_tokens()

        thr_track_items = self.cw.get_thr_track_items()

        for (func, id) in thr_track_items:
            calling_func_list = self.fm.get_calling_functions_list(func, analysis_dict)
            self.gh.track_thread_items(calling_func_list, id)

        self.cw.clear_thr_track_tokens()

        local_track_items = self.cw.get_local_track_items()
        spin_locations = self.cw.get_spin_locations()

        spin_funcs = set()
        for func in spin_locations:
            calling_func_list = self.fm.get_calling_functions_list(func, analysis_dict)
            for (calling_func, name) in calling_func_list:
                spin_funcs.add(calling_func)

        for (func, id) in local_track_items:
            if func not in spin_funcs:
                self.gh.mark_graph_node_as_local(id)

        self.cw.clear_spin_locations()
        self.cw.clear_local_track_tokens()
 

    def post_process_collect_from_file_group(self):
        for (calling_func_list, id_string) in self.cb_link_track_items:
            for calling_func in calling_func_list:
                is_cb = self.gh.is_callback_function(calling_func)
                if is_cb:
                    self.gh.track_potential_callback_link(calling_func, id_string)

        self.cb_link_track_items.clear()

        # ASSOCIATE CALLBACK GROUPS OF A NODE TO THE NODE'S EXECUTOR
        self.gh.set_executor_callback_groups()

        # ADD EDGES IF SPIN INVOKED FROM CALLBACK
        self.gh.find_and_update_callback_spin_links()

        self.gh.save_callback_function_dict()
        self.gh.reset_callback_function_dict()


    def post_process_collect(self):
        self.gh.setup_callback_links()
        self.gh.assign_threads()


    def post_process_collect_from_py(self):
        init_func_scopes = self.pw.get_init_func_scopes()

        # SHOULD BE CALLED ONLY AFTER PY WALKER COLLECT
        self.sm.filter_function_graph(init_func_scopes)

        self.pw.clear_init_func_scopes()

        cb_link_track_items = self.pw.get_cb_link_track_items()

        for (func_scope, id_string) in cb_link_track_items:
            calling_func_list = self.sm.get_calling_functions_list(func_scope)
            self.cb_link_track_items.append((calling_func_list, id_string))

        self.pw.clear_cb_link_track_tokens()

        thr_track_items = self.pw.get_thr_track_items()

        for (func_scope, id) in thr_track_items:
            calling_func_list = self.sm.get_calling_functions_list(func_scope)
            self.gh.track_thread_items(calling_func_list, id)

        self.pw.clear_thr_track_tokens()

        local_track_items = self.pw.get_local_track_items()
        spin_locations = self.pw.get_spin_locations()

        spin_funcs = set()
        for func_scope in spin_locations:
            calling_func_list = self.sm.get_calling_functions_list(func_scope)
            for (calling_func, name) in calling_func_list:
                spin_funcs.add(calling_func['sscope'])

        for (func_scope, id) in local_track_items:
            if func_scope['sscope'] not in spin_funcs:
                self.gh.mark_graph_node_as_local(id)

        self.pw.clear_spin_locations()
        self.pw.clear_local_track_tokens()


    def print_filegroup_to_file(self, group, errors_file='errors.txt', restart=False):
        mode = 'w' if restart else 'a'

        with open(errors_file, mode) as f:
            f.write("\n-")
            for group_file in group:
                f.write("\n%s" % (group_file))
            f.write("\n-")
    

    def display_graph(self, graph_name='output/cbexec_graph.png'):
        # output_dir = os.getcwd() + "output"
        if not os.path.exists('output'):
            os.mkdir('output')
        self.gh.print_graph(graph_name)



class ImportVisitor(ast.NodeVisitor):

    def __init__(self) -> None:
        self.ros_version = None


    def visit_Import(self, astnode: ast.Import):
        for name in astnode.names:
            if name.name == 'rclpy':
                self.ros_version = 2
            elif name.name == 'rospy':
                self.ros_version = 1

            
    def visit_ImportFrom(self, astnode: ast.ImportFrom):
        fromname = astnode.module
        if fromname:
            if fromname == 'rclpy':
                self.ros_version = 2
            elif fromname == 'rospy':
                self.ros_version = 1


    def get_ros_version(self):
        return self.ros_version



