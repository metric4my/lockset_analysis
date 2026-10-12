#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#All rights reserved.


from utils import find_files
import os
import traceback
import copy
import ast
import parsing as CP
import lxml.etree as ET
import py_ast_util as pau


# TRY FOLLOWING PACKAGE INSTEAD OF ABOVE: cmakeast


class FileGroupsManager:

    def __init__(self):
        self.current_ros_version = 1
        self.current_py_ros_version = 1
        self.pkg_dir_dict = {} # pkgname: pkgdir
        self.node_files_dict = {} # node_exe : cpp_files, ros_version, is_linked
        self.global_variables = {}
        self.pending_link_libs = {}
        self.node_script_files_dict = {}
        self.launch_dict = {} # launch_path: launch_files, nodes


    def get_all_file_groups(self, target):
        self.parse_cmake(target)
        ##TODO process for pending libraries after each package or whole repository??
        ##TODO for now, process after the whole repository
        self.process_pending_links()

        pynode_files_dict = self.parse_setup(target)
        
        groups = {}
        for name, info in self.node_files_dict.items():
            if info['is_linked'] == False:
                groups[name] = info
        for name, info in self.node_script_files_dict.items():
            groups[name] = info
        for name, info in pynode_files_dict.items():
            groups[name] = info
        return groups
    

    def get_file_groups(self, target, launch_path=''):
        if not launch_path:
            return self.get_all_file_groups(target)
        
        self.parse_pkg(target)
        for pkgname, pkgdir in self.pkg_dir_dict.items():
            self.parse_cmake(pkgdir)
        ##TODO process for pending libraries after each package or whole repository??
        ##TODO for now, process after the whole repository
        self.process_pending_links()

        pynode_files_dict = self.parse_setup(target)

        self.parse_launch_file(launch_path)

        # RETURN ALL NODES IN LAUNCH_DICT AS IT CONTAINS ENTRIRES FROM LAUNCH_PATH ONLY
        ##TODO store nodes list instead of launch dict and use the list directlry, as we are processing only one given launch file?
        nodes = []
        for lfile, lcontent in self.launch_dict.items():
            nodes.extend(lcontent['nodes'])
        nodes = list(set(nodes))

        groups = {}
        for name in nodes:
            info = self.node_files_dict.get(name, None)
            if info:
                if info['is_linked'] == False:
                    groups[name] = info
            else:
                info = self.node_script_files_dict.get(name, None)
                if info:
                    groups[name] = info
                else:
                    info = pynode_files_dict.get(name, None)
                    if info:
                        groups[name] = info        
        return groups
    

    def parse_pkg(self, target):
        for pfile in find_files(target, 'package.xml'):
            try:
                parser = ET.XMLParser(ns_clean=True,recover=True)
                tree = ET.parse(pfile, parser)
                root = tree.getroot()
                if root.tag == 'package':
                    for child in root:
                        if child.tag == 'name':
                            self.add_pkg(pfile, child.text)
                        break        

            except ET.ParseError:
                print("Tree parse error in file %s" % pfile)

            except UnicodeEncodeError:
                print("Unicode error")
      
            except Exception as error:
                traceback.print_exc()
                print(error)


    def add_pkg(self, pfile, pkgname):
        self.pkg_dir_dict[pkgname] = os.path.dirname(pfile)


    def parse_setup(self, target):
        pynode_files_dict = {}

        for sfile in find_files(target, 'setup.py'):
            with open(sfile, 'r') as fp:
                scode = fp.read()
                tree = ast.parse(scode)
                relpath = os.path.relpath(os.path.dirname(sfile), target)
                relpath = '' if relpath == '.' else relpath

                setup_visitor = PYFileVisitor()
                nf_dict = setup_visitor.parse_from_setup_tree(tree, relpath)
                if nf_dict:
                    pynode_files_dict.update(nf_dict)

        return pynode_files_dict


    def parse_cmake(self, target):
        self.global_variables.clear()
        ##TODO process for pending libraries after each package or whole repository??
        ##TODO for now, process after the whole repository
        #self.pending_link_libs.clear()

        for cfile in find_files(target, 'cmakelists.txt'):
            if not os.path.exists(cfile):
                # THIS IS A SYMLINK WITH TARGET THAT DOES NOT EXIST
                continue

            try:
                with open(cfile) as f:
                    contents = f.read()
                    tree = CP.parse(contents)
                    relpath = os.path.relpath(os.path.dirname(cfile), target)
                    relpath = '' if relpath == '.' else relpath
                    self.current_ros_version = 1
                    self.parse_from_cmake_tree(tree, relpath)

            #except AttributeError:
            #    print("Attibute error")

            except Exception as error:
                traceback.print_exc()
                print(error)
                print("WARNING: cmake parse failed")

        ##TODO process for pending libraries after each package or whole repository??
        ##TODO for now, process after the whole repository
        #self.process_pending_links()


    def parse_from_cmake_tree(self, root, relpath):
        # TREE IS A LIST OF COMMANDS

        #print("\nGLOBAL_VARIABLES: {gvars}\n".format(gvars=self.global_variables))
        variables = copy.deepcopy(self.global_variables)

        for child in root:
            if isinstance(child, CP._Command):
                cmd = child.name.lower()
                args = child.body

                if cmd == 'add_executable' or cmd == 'add_library':
                    exe_name = self.get_cmake_name(args[0].contents, variables)
                    if not exe_name:
                        continue
                    cpp_files = []
                    for arg in args[1:]:
                        name = self.get_cmake_name(arg.contents, variables)
                        if not name:
                            continue
                        if isinstance(name, list):
                            cpp_files.extend(name)
                        else:
                            name = os.path.join(relpath, name)
                            cpp_files.append(name)
                    self.add_cmake_exe(exe_name, cpp_files)

                elif cmd == 'target_link_libraries':
                    exe_name = self.get_cmake_name(args[0].contents, variables)
                    if not exe_name:
                        continue
                    cpp_files = []
                    libs = []
                    for arg in args[1:]:
                        name = self.get_cmake_name(arg.contents, variables, default_None=False)
                        ##TODO process each element of the list instead?
                        if isinstance(name, list):
                            continue
                        if name in self.node_files_dict:
                            cpp_files.extend(self.node_files_dict[name]['cpp_files'])
                            self.node_files_dict[name]['is_linked'] = True
                        else:
                            # PROCESS AFTER ALL CMAKELISTS.TXT FILES ARE PARSED
                            libs.append(name)
                    self.add_cmake_exe(exe_name, cpp_files)
                    self.track_pending_link_libs(exe_name, libs)

                elif cmd == 'project':
                    name = self.get_cmake_name(args[0].contents, variables)
                    if name:
                        variables['PROJECT_NAME'] = name

                elif cmd == 'set':
                    var_name = self.get_cmake_name(args[0].contents, variables)
                    if not var_name:
                        continue
                    value = []
                    for arg in args[1:]:
                        name = self.get_cmake_name(arg.contents, variables)
                        if not name:
                            continue
                        if isinstance(name, list):
                            value.extend(name)
                        else:
                            name = os.path.join(relpath, name)
                            value.append(name)
                    variables[var_name] = value

                elif cmd == 'find_package':
                    name = self.get_cmake_name(args[0].contents, variables)
                    if name:
                        if name == 'rclcpp':
                            self.current_ros_version = 2
                        elif name == 'rclpy':
                            self.current_py_ros_version = 2
                        elif name == 'ament_cmake':
                            self.current_py_ros_version = 2

                elif cmd == 'catkin_package':
                    if 'PROJECT_NAME' not in variables:
                        continue
                    lib_mode = False
                    value = []
                    for arg in args:
                        name = self.get_cmake_name(arg.contents, variables)
                        if not name:
                            continue
                        if name == 'LIBRARIES':
                            lib_mode = True
                        elif lib_mode:
                            if isinstance(name, list):
                                value.extend(name)
                            elif name in ['CATKIN_DEPENDS', 'DEPENDS', 'INCLUDE_DIRS']:
                                lib_mode = False
                            else:
                                value.append(name)
                    var_name = variables['PROJECT_NAME'] + '_LIBRARIES'
                    variables[var_name] = value
                    self.global_variables[var_name] = value

                elif cmd == 'install' or cmd == 'catkin_install_python':
                    prog_mode = False
                    for arg in args:
                        name = self.get_cmake_name(arg.contents, variables)
                        if not name:
                            continue
                        if name == 'PROGRAMS':
                            prog_mode = True
                        elif name == 'DESTINATION':
                            prog_mode = False
                        elif prog_mode:
                            ##TODO differentiate between python and other scripts when install command is used
                            ##TODO currently, assume that it is a python script
                            script_name = os.path.basename(name)
                            name = os.path.join(relpath, name)
                            py_files = [name]
                            self.add_cmake_script(script_name, py_files)


    def get_cmake_name(self, name, variables, default_None=True):
        idx1 = name.find('${')
        idx2 = name.find('}', idx1)
        if idx1 != -1 and idx2 != -1:
            var_name = name[idx1+2:idx2]

            if var_name in variables:
                value = variables[var_name]
                if isinstance(value, list):
                    if len(value) > 1:
                        return value
                    elif len(value) == 1:
                        value = value[0]
                    else:
                        return None if default_None else name
                new_name = name[:idx1] + value + name[idx2+1:]      
                return new_name
            else:
                return None if default_None else name
        else:
            return name


    def add_cmake_exe(self, exe_name, cpp_files):
        if exe_name in self.node_files_dict:
            self.node_files_dict[exe_name]['cpp_files'].extend(cpp_files)
        else:
            self.node_files_dict[exe_name] = {'cpp_files': cpp_files,
                                              'ros_version': self.current_ros_version,
                                              'is_linked': False}
            

    def track_pending_link_libs(self, exe_name, libs):
        if exe_name in self.pending_link_libs:
            self.pending_link_libs[exe_name].extend(libs)
        else:
            self.pending_link_libs[exe_name] = libs


    def process_pending_links(self):
        #print("\nGLOBAL_VARIABLES: {gvars}\n".format(gvars=self.global_variables))
        #print("\nPENDING: {pending}\n".format(pending=self.pending_link_libs))

        for exe_name, libs in self.pending_link_libs.items():
            cpp_files = []
            for l in libs:
                name = self.get_cmake_name(l, self.global_variables)
                if name in self.node_files_dict:
                    cpp_files.extend(self.node_files_dict[name]['cpp_files'])
                    self.node_files_dict[name]['is_linked'] = True
            if cpp_files:
                self.add_cmake_exe(exe_name, cpp_files)


    def add_cmake_script(self, script_name, py_files):
        if script_name in self.node_script_files_dict:
            self.node_script_files_dict[script_name]['py_files'].extend(py_files)
        else:
            self.node_script_files_dict[script_name] = {'py_files': py_files,
                                                        'ros_version': self.current_py_ros_version}

    
    def parse_launch_file(self, launch_path):
        try:
            parser = ET.XMLParser(ns_clean=True,recover=True)
            tree = ET.parse(launch_path, parser)

            launch_files, node_exes = self.parse_from_xml_tree(tree.getroot())
            self.add_launch_file(launch_path, launch_files, node_exes)
            for lfile in launch_files:
                self.parse_launch_file(lfile)
     
        except ET.ParseError:
            print("Tree parse error in file %s" % launch_path)

        except UnicodeEncodeError:
            print("Unicode error")
      
        except Exception as error:
            traceback.print_exc()
            print(error)


    def parse_from_xml_tree(self, root):
        launch_files = []
        node_exes = []

        for child in root:
            if len(child) >= 2:
                (l,n) = self.parse_from_xml_tree(child)
                launch_files = launch_files + l
                node_exes = node_exes + n

            if child.tag == "include" and 'file' in child.attrib:
                file_name = child.attrib['file']
                resolved_name = self.resolve_args(file_name)
                if resolved_name:
                    #file_path = os.path.join(self.target + resolved_name)
                    #if os.path.exists(file_path):
                        #launch_files.append(file_path)
                    launch_files.append(resolved_name)
            elif child.tag == "node" and 'pkg' in child.attrib and 'type' in child.attrib:
                pkg_name = child.attrib['pkg']
                if pkg_name not in self.pkg_dir_dict:
                    #print "Non-local package node %s" % (pkg_name)
                    continue

                exe_name = child.attrib['type']
                if exe_name not in self.node_files_dict:
                    #print "Executable %s not found" % (exe_name)
                    continue

                self.mark_node_as_used(exe_name)
                node_exes.append(exe_name)                
                    
        return launch_files, node_exes
    

    def resolve_args(self, name):
        if '$' not in name:
            return name

        # FIND ARG
        idx1 = name.find('$(')
        if idx1 == -1:
            return None

        idx2 = name.find(')', idx1)
        if idx2 == -1:
            return None

        start = name[:idx1]
        arg = name[idx1+2:idx2]
        end = name[idx2+1:]

        parts = arg.split(' ')
        parts = [p for p in parts if p]
        if len(parts) != 2 or parts[0] != 'find':
            return None

        pkg_name = parts[1]
        if pkg_name not in self.pkg_dir_dict:
            #print "Non-local package in launch file name %s" % (name,)
            return None

        return start + self.pkg_dir_dict[pkg_name] + end


    def add_launch_file(self, launch_path, launch_files, node_exes):
        self.launch_dict[launch_path] = {'launch_files': launch_files,
                                         'nodes': node_exes}
                 


class PYFileVisitor(ast.NodeVisitor):

    def __init__(self) -> None:
        super().__init__()

        ##TODO support python node with multiple source files
        # currently supports only single file python nodes
        self.node_files_dict = {} # node_exe : py_files, ros_version
        self.current_relpath = None


    def parse_from_setup_tree(self, root, relpath):
        self.current_relpath = relpath
        self.visit(root)
        return self.node_files_dict

    
    def add_py_exe(self, exe_name, py_files):
        if exe_name in self.node_files_dict:
            self.node_files_dict[exe_name]['py_files'].extend(py_files)
        else:
            self.node_files_dict[exe_name] = {'py_files': py_files,
                                              'ros_version': 2} ##TODO support ros 1

    
    def visit_Call(self, astnode: ast.Call):
        func_name = pau.get_name(astnode.func)

        if func_name == 'setup':
            node_dict = pau.get_arg_at_pose_from_left(astnode, 'entry_points', 99) # big pos number so that it is not processed
            node_list = pau.get_dict_value_for_key(node_dict, 'console_scripts')
            if isinstance(node_list, ast.List):
                for elt in node_list.elts:
                    node_str = pau.get_string_value(elt)
                    if node_str:
                        exe_name, part2 = node_str.split('=')
                        pkg_name, part2 = part2.split('.', 1) if ('.' in part2) else ('', part2)
                        file_name, part2 = part2.split(':')
                        file_path = os.path.join(self.current_relpath, pkg_name.strip(), file_name.strip().replace('.', '/') + '.py')
                        self.add_py_exe(exe_name.strip(), [file_path])


