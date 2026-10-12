#Copyright 2025 Purdue University.
#All rights reserved.


import symtable
import ast
import sys
from enum import Enum

from scopes_manager import ScopesManager
import py_ast_util as pau

this = sys.modules[__name__]

this.NUM = Enum('NUM', ['UNKNOWN'])


class PYSymbolHelper:

    def __init__(self, sm) -> None:
        self.sm = sm #ScopesManager()
        self.st = None
        self.current_st = None
        # symtable package has self symbol in every function that assigns / uses self.<variable>
        # following is a workaround to propagate self variables's info across functions in a class.
        self.current_class_symbols = {}


    # workaround for self variables
    def update_current_class_symbols(self, current_scope_info):
        class_sym = self.get_symbol('self', current_scope_info, recursive=False)
        if class_sym and hasattr(class_sym, 'val'):
            self.current_class_symbols.update(class_sym.val)


    # workaround for self variables
    def reset_current_class_symbols(self):
        self.current_class_symbols.clear()


    def generate_symbol_table(self, source):
        self.st = symtable.symtable(source, "", "exec")
        self.current_st = self.st


    def get_symbol(self, name, scope_info, recursive=True):
        if not name or not scope_info:
            return None
        
        if not isinstance(scope_info, tuple):
            return None

        (stype, sname) = scope_info
        scope = self.sm.get_scope(stype, sname)
        
        sym = None
        while(scope):
            sym = self.sm.lookup_name_in_scope(name, scope)
            if sym:
                break

            if not recursive:
                break

            scope = self.sm.get_parent(scope)

        return sym
    

    def is_local_symbol(self, sym):
        if sym.get_name() == 'self':
            return False
        else:
            return sym.is_local()
        

    def is_parameter_symbol(self, sym):
        return sym.is_parameter()


    def set_val(self, sym, new_key, new_val):
        if hasattr(sym, 'val'):
            sym.val[new_key] = new_val
        else:
            sym.val = {new_key: new_val}


    def get_val(self, sym, val_key):
        value = None
        if hasattr(sym, 'val'):
            value = sym.val.get(val_key)

        if (not value) and sym.get_name() == 'self':
            # workaround for self variables
            value = self.current_class_symbols.get(val_key)
        
        return value

        
    # parent_info to distinguish between same function names under diffrent scopes
    def get_function_object(self, scope_info, parent_info):
        # return the scope manager entry
        current_type, current_name = scope_info
        function_scope_obj = self.sm.get_scope(current_type, current_name)

        if parent_info:
            parent_type, parent_name = parent_info
            parent_scope_obj = self.sm.get_parent(function_scope_obj)
            if parent_scope_obj and \
                    self.sm.get_type(parent_scope_obj) == parent_type and self.sm.get_name(parent_scope_obj) == parent_name:
                return function_scope_obj
            else:
                return None
        else: # no parent scope
            return function_scope_obj
        

    def get_callback_function(self, func_name):
        if not func_name:
            return None     
        func_obj = self.get_function_object(('function', func_name), parent_info=None)
        return (func_obj, func_name)



def set_val(astnode, new_key, new_val):
    if hasattr(astnode, 'val'):
        astnode.val[new_key] = new_val
    else:
        astnode.val = {new_key: new_val}


def get_val(astnode, val_key):
    if hasattr(astnode, 'val'):
        return astnode.val.get(val_key)
    else:
        return None


def get_arg_count(callnode):
    return pau.get_arg_count(callnode)


def get_arg_at_pose_from_left(astnode, arg_name, pos):
    return pau.get_arg_at_pose_from_left(astnode, arg_name, pos)


def get_name(astnode):
    return pau.get_name(astnode)


def get_object_and_name(astnode):
    if astnode:
        if isinstance(astnode, ast.Name):
            return (None, astnode.id)
        elif isinstance(astnode, ast.Attribute):
            return (astnode.value, astnode.attr)

    return (None,'')


def get_string_value(astnode):
    return pau.get_string_value(astnode)


def get_dict_value_for_key(astnode, d_key):
    return pau.get_dict_value_for_key(astnode, d_key)


def get_tuple_value_at_pose(astnode, pos):
    return pau.get_tuple_value_at_pose(astnode, pos)


def get_numeric_value(astnode):
    if isinstance(astnode, ast.Constant):
        value = astnode.value
    else:
        value = this.NUM.UNKNOWN

    return value


def get_time_value(astnode):
    value = this.get_numeric_value(astnode)
    if value != this.NUM.UNKNOWN:
        return value
    
    ##TODO process other time values such as rate, duration

    return value


def get_callback_function_name(astnode, current_class_scope_name):
    if isinstance(astnode, ast.Call):
        (func_obj, func_str) = this.get_object_and_name(astnode.func)
        if func_str == 'partial': #functools.partial function
            astnode = this.get_arg_at_pose_from_left(astnode, '', 1)

    cb_func_name = this.get_name(astnode)
    if cb_func_name and cb_func_name.startswith('self.'):
        cb_func_name = cb_func_name.replace('self.', current_class_scope_name + '::', 1)
    return cb_func_name
