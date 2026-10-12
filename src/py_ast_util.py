#Copyright 2025 Purdue University.
#All rights reserved.


import ast


def get_arg_count(callnode):
    return len(callnode.keywords) + len(callnode.args)


def get_arg_at_pose_from_left(astnode, arg_name, pos):
    arg = None

    if isinstance(astnode, ast.Call):
        if arg_name:
            for k in astnode.keywords:
                if k.arg == arg_name:
                    arg = k.value

        if not arg and len(astnode.args) >= pos:
            arg = astnode.args[pos-1]

    return arg


def get_name(astnode):
    name = ''

    if not astnode:
        return name
    elif isinstance(astnode, ast.Name):
        name = astnode.id
    elif isinstance(astnode, ast.Attribute):
        name = get_name(astnode.value)
        name = name + '.' + astnode.attr

    return name


def get_string_value(astnode):
    if isinstance(astnode, ast.Constant):
        value = astnode.value
    else:
        value = None

    return value


def get_dict_value_for_key(astnode, d_key):
    value = None

    if isinstance(astnode, ast.Dict):
        i = 0
        for key in astnode.keys:
            if isinstance(key, ast.Constant) and get_string_value(key) == d_key:
                value = astnode.values[i]
                break
            i = i+1
            
    return value


def get_tuple_value_at_pose(astnode, pos):
    value = None

    if isinstance(astnode, ast.Tuple):
        if len(astnode.elts) >= pos:
            value = astnode.elts[pos-1]

    return value
