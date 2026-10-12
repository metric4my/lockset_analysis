#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


import sys
from enum import Enum

this = sys.modules[__name__]

this.NUM = Enum('NUM', ['UNKNOWN'])

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# HELPER FUNCTIONS
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

''' return: (func_obj, func_str)
    '''
def get_thread_function(token):
    return get_callback_function(token)


''' return: (func_obj, func_str)
    '''
def get_callback_function(token):
    if this.is_match(token, '(') and this.is_match_previous(token, ['::','bind']):
        paren_token = token
        arg_cnt = this.get_count_of_function_arguments(paren_token)
        func_token = this.get_comma_separated_token_at_pose_from_right(paren_token, arg_cnt)
        ##TODO get the arg_cnt-1 token for the object whose member functions is a callback
        ##TODO get the function object, as cppcheck ast does not have it for the func token in std::bind?
        if this.is_match(func_token, '&'):
            func_token = func_token.astOperand1
        return (None, this.recursively_visit(func_token))
    elif this.is_match(token, '&'):
        func_token = token.astOperand1
        return (None, this.recursively_visit(func_token))
    ##TODO support lambda functions
    elif token and token.function:
        func_token = token
        return (token.function, this.recursively_visit(func_token))
    elif token:
        ##TODO get the function object from the sorted analysis dict or function graph, when there is no function associated with token?
        return (None, this.recursively_visit(token))
    else:
        return None
    

def get_numeric_value(token):
    ##TODO support numeric/boolean variables val assignment and val propagation
    """ token_var, token_str = self.get_var_token_and_var_name(token)
        if (token_var and token_var.variable and token_var.variable.val):
            value = token_var.variable.val.get('num')
        elif token.val:
            value = token.val.get('num')
        else:
            value = self.gh.NUM.UNKNOWN
        """
    if token.isInt:
        value = int(token.str)
    elif token.isFloat:
        value = float(token.str)
    elif token.str == 'true':
        value = 1
    elif token.str == 'false':
        value = 0
    else:
        value = this.NUM.UNKNOWN

    return value


def convert_string_to_number(s):
    value = this.NUM.UNKNOWN

    temp = s.replace('-', '') if s.startswith('-') else s
    if temp.isnumeric():
        value = int(s)
    else:
        temp = temp.replace('.', '')
        if temp.isnumeric():
            value = float(s)
            
    return value


def get_time_value(token):
    value = this.get_numeric_value(token)
    if value != this.NUM.UNKNOWN:
        return value

    ##TODO support boost::chrono, std::chrono duration typedefs 
    if token.str == '(' and this.is_match_previous(token, ['chrono','::','nanoseconds']):
        value = this.get_numeric_value(token.astOperand2)
        if value != this.NUM.UNKNOWN:
            value = value * 0.000000001
    elif token.str == '(' and this.is_match_previous(token, ['chrono','::','microseconds']):
        value = this.get_numeric_value(token.astOperand2)
        if value != this.NUM.UNKNOWN:
            value = value * 0.000001
    elif token.str == '(' and this.is_match_previous(token, ['chrono','::','milliseconds']):
        value = this.get_numeric_value(token.astOperand2)
        if value != this.NUM.UNKNOWN:
            value = value * 0.001
    elif token.str == '(' and this.is_match_previous(token, ['chrono','::','seconds']):
        value = this.get_numeric_value(token.astOperand2)
    elif token.str == '(' and this.is_match_previous(token, ['chrono','::','minutes']):
        value = this.get_numeric_value(token.astOperand2)
        if value != this.NUM.UNKNOWN:
            value = value * 60
    elif token.str == '(' and this.is_match_previous(token, ['chrono','::','hours']):
        value = this.get_numeric_value(token.astOperand2)
        if value != this.NUM.UNKNOWN:
            value = value * 3600

    ##TODO support ros::Duration, ros::WallDuration, ros::Rate, ros::WallRate
    elif token.str == '(' and token.previous and token.previous.str.endswith('Rate') and this.is_match_previous(token.previous, ['ros','::']):
        if this.is_match(token.astOperand2, '('): # Duration
            value = this.get_time_value(token.astOperand2)
        else:
            value = this.get_numeric_value(token.astOperand2)
            if value != this.NUM.UNKNOWN: # Hz
                value = 1.0 / value 
    elif token.str == '(' and token.previous and token.previous.str.endswith('Duration') and this.is_match_previous(token.previous, ['ros','::']):
        if this.is_match(token.astOperand2, '('): # Rate
            value = this.get_time_value(token.astOperand2)
        elif this.is_match(token.astOperand2, ','): # Sec, NSec
            value = this.NUM.UNKNOWN
            snum = this.get_numeric_value(token.astOperand2.astOperand1) if token.astOperand2.astOperand1 else None
            nsnum = this.get_numeric_value(token.astOperand2.astOperand2) if token.astOperand2.astOperand2 else None
            if snum and nsnum and snum != this.NUM.UNKNOWN and nsnum != this.NUM.UNKNOWN:
                value = snum + nsnum * 0.
        elif token.astOperand2: # sec
            value = this.get_numeric_value(token.astOperand2)
        else: # constructor without arg
            value = 0

    ##TODO support std::chrono::duration numbers e.g. 1s (h,min,s,ms,us,ns)
    elif token.str.endswith('ns'):
        s = token.str[:-2]
        value = this.convert_string_to_number(s)
    elif token.str.endswith('us'):
        s = token.str[:-2]
        value = this.convert_string_to_number(s)
    elif token.str.endswith('ms'):
        s = token.str[:-2]
        value = this.convert_string_to_number(s)
    elif token.str.endswith('s'):
        s = token.str[:-1]
        value = this.convert_string_to_number(s)
    elif token.str.endswith('min'):
        s = token.str[:-3]
        value = this.convert_string_to_number(s)
    elif token.str.endswith('h'):
        s = token.str[:-1]
        value = this.convert_string_to_number(s)

    return value


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# VAL DICTIONARY HELPER FUNCTIONS
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def get_val_id(token):
    val_id = ''
    (t_var, t_str) = this.get_var_token_and_var_name(token)
    if t_var and t_var.variable and t_var.variable.val:
        val_id = t_var.variable.val.get('id', '')
    elif token.val:
        val_id = token.val.get('id', '')
    return val_id


def update_val_dict(val_dict, new_key, new_val):
    if val_dict:
        val_dict[new_key] = new_val
    else:
        val_dict = {new_key: new_val}
    return val_dict


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# AST HELPER FUNCTIONS
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -    

''' GENERIC RECURSION PATTERN - LEFT RIGHT TOKEN
    input:  token  CPPCHECK token to recurse upon
    function_to_apply:  the function to apply to the token after recursing.
    returns: None   Side effect determined by function_to_apply
    '''
def generic_recurse_and_apply_function(token, function_to_apply):
    if not token:
        return

    # INITIALIZE
    left_token = right_token = None

    # LEFT
    if token.astOperand1:
        left_token = token.astOperand1
        this.generic_recurse_and_apply_function(left_token,
                                                function_to_apply)
    # RIGHT
    if token.astOperand2:
        right_token = token.astOperand2
        this.generic_recurse_and_apply_function(right_token,
                                                function_to_apply)

    function_to_apply(token, left_token, right_token)


def is_match(token, value):
    return True if (token and token.str == value) else False


def is_match_previous(token, val_list):
    for val in reversed(val_list):
        if token.previous and token.previous.str == val:
            token = token.previous
        else:
            return False
    return True


def is_match_next(token, val_list):
    for val in val_list:
        if token.next and token.next.str == val:
            token = token.next
        else:
            return False
    return True


def get_comma_separated_token_at_pose_from_right(paren_token, pos):
    if this.is_match(paren_token, '('):
        token = paren_token.astOperand2
        i = 1
        while i < pos:
            if not token:
                return None
            token = token.astOperand1
            i += 1

        if i == pos and token:
            return token.astOperand2 if this.is_match(token, ',') else token
    
    return None


def recurse_and_get_comma_separated_tokens(token, arguments):
    if not token:
        return

    if token.str == ',':
        this.recurse_and_get_comma_separated_tokens(token.astOperand1, arguments)
        this.recurse_and_get_comma_separated_tokens(token.astOperand2, arguments)
    else:
        arguments.append(token)


def get_function_arguments(paren_token):
    arguments = []
    if this.is_match(paren_token, '('):
        this.recurse_and_get_comma_separated_tokens(paren_token.astOperand2, arguments)

    return arguments


def get_count_of_function_arguments(paren_token):
    if this.is_match(paren_token, '('):
        token = paren_token.astOperand2
        if not token:
            return 0

        i = 1
        while token.str == ',':
            i += 1
            token = token.astOperand1

        return i
    

def get_variable_tokens_and_name(token):
    while token.astParent and token.astParent.str in ['.']:
        token = token.astParent
    return (this.get_variable_tokens(token), this.recursively_visit(token))


def get_variable_tokens(token):
    variable_tokens = []

    # LEFT RECURSE
    if token.astOperand1:
        variable_tokens.extend(this.get_variable_tokens(token.astOperand1))
    
    # SELF
    if token.variable:
        variable_tokens.append(token)

    # RIGHT_RECURSE
    if token.astOperand2:
        variable_tokens.extend(this.get_variable_tokens(token.astOperand2))

    return variable_tokens


def get_var_token_and_var_name(token):
    if this.is_match(token, '::') and token.astOperand2:
        # INIT STATEMENT e.g. const std::string s = 's'
        token = token.astOperand2

    if this.is_match(token, '*') and token.astOperand1 and (not token.astOperand2):
        # DEREFERENCE
        token = token.astOperand1

    var_token = token
    var_name = token.str
    
    if token.str in ['.', '[', '(', '&']:
        (var_token, var_name) = this.find_compound_variable_token_and_name_for_sym_token(token)

    return (var_token, var_name)


#TODO check which variable token to be returned
def find_compound_variable_token_and_name_for_sym_token(token):
    if token.str not in ['.', '[', '(', '&']:
        print('received a non symbol token for tokenid:%s str:%s' % (token.Id, token.str))
        return (token, token.str)
    
    ##TODO do we need to check for this?
    ros_functions_with_types = ['x', 'y', 'z', 'w', 'getX', 'getY', 'getZ', 'getW', 'getOrigin', 'getRotation', 'getBasis']

    compound_variable_token = token
    if token.str == '(' and token.astOperand1:
        compound_variable_token = token.astOperand1
    elif token.str == '&' and token.astOperand2:
        compound_variable_token = token.astOperand2

    while compound_variable_token.astOperand1 and (not compound_variable_token.variable):
        if compound_variable_token.str not in ['[', '('] and \
                compound_variable_token.astOperand2 and compound_variable_token.astOperand2.variable:
            compound_variable_token = compound_variable_token.astOperand2
        else:
            compound_variable_token = compound_variable_token.astOperand1

    if token.str == '(':
        if compound_variable_token.variable or compound_variable_token.varId:
            #var_type = self.find_variable_type(compound_variable_token.variable)
            #var_type = var_type.lower()
            # Remove the following check to support variable length arrays (~std::vector in c++) of ros types' members
            #((any(substr in var_type for substr in ['vector','deque','queue','map']) and \               
            if token.astOperand1 and token.astOperand1.str == '.' and \
                    token.astOperand1.astOperand2 and \
                    (token.astOperand1.astOperand2.str in ['at', 'back', 'front'] or \
                    token.astOperand1.astOperand2.str in ros_functions_with_types):
                name = this.recursively_visit(token.astOperand1.astOperand1)
                return (compound_variable_token, name)

        # FUNCTION CALL
        return (token, token.str)

    if token.str == '&' and token.astOperand2:
        token = token.astOperand2
        
    name = this.recursively_visit(token)
    return (compound_variable_token, name)


''' input: a cppcheckdata token
    returns: string aggregation of tokens under the root
    '''
def recursively_visit(token):
    my_return_string = ''

    if token.astOperand1: 
        my_return_string += this.recursively_visit(token.astOperand1)
    my_return_string += token.str 
    if token.astOperand2:
        my_return_string += this.recursively_visit(token.astOperand2)
    if token.str == '[':
        my_return_string += ']'
    elif token.str == '(':
        my_return_string += ')'
    
    return my_return_string


def is_local_symbol(variable):
    return (variable and variable.isLocal and (not variable.isStatic))


def get_class_scope_name_for_function(function):
    func_def_token = function.tokenDef
    if func_def_token.scope and func_def_token.scope.type == 'Class':
        return func_def_token.scope.className
    else:
        return None


def get_class_scope_name_for_token(token):
    if token.scope and token.scope.type == 'Function' and token.scope.function:
        return this.get_class_scope_name_for_function(token.scope.function)
    else:
        return None
    

def get_function_for_token(token):
    scope = token.scope
    while scope:
        if (scope.type == 'Function' or scope.type == 'Lambda') and scope.function:
            return scope.function
        scope = scope.nestedIn
    return None
        

def find_sanitized_variable_type(variable):
    var_type = this.find_variable_type(variable)
    # SANITIZE VAR TYPE
    var_type = this.sanitize_class_name(var_type)
    return var_type


''' input: cppcheckdata variable object
    output: string of variable type  (ex:  'int'  or 'std::vector') 
    '''
def find_variable_type(variable):
    my_return_string = variable.typeStartToken.str

    if variable.typeStartToken != variable.typeEndToken:
        nextToken = variable.typeStartToken.next
        hasNext = True
        while(hasNext and nextToken):  # BREAK at end of type region
            my_return_string += nextToken.str
            if nextToken.Id == variable.typeEndTokenId:
                hasNext = False;
            nextToken = nextToken.next

    return my_return_string


def rreplace(s, old, new, occurance):
    li = s.rsplit(old, occurance)
    return new.join(li)


''' TAKES A CLASS NAME AND STRIPTS EXTRA CLASS INFORMATION FROM FRONT AND BACK
    input: class_name 'constPtr<tf2::Transform::iterator>'
    output: str       'trf2::Transform' 
    '''
def sanitize_class_name(class_name):       
    is_some_change = True
    while (is_some_change):# SOME OF THESE CAN BE CASCADED IN DIFFERENT ORDERS
        is_some_change = False

        # SIMPLIFY ITERATORS
        if str(class_name).endswith('::iterator'):
            class_name = this.rreplace(class_name, '::iterator', '', 1)
            is_some_change = True
        if str(class_name).endswith('::const_iterator'):
            class_name = this.rreplace(class_name, '::const_iterator', '', 1)
            is_some_change = True
        # SIMPLIFY REFERNCES
        if str(class_name).endswith('&'):
            class_name = class_name.replace('&', '')
            is_some_change = True
        # SIMPLIFY CONST
        if str(class_name).endswith('const') or str(class_name).startswith('const'):
            class_name = this.rreplace(class_name, 'const', '', 1)
            is_some_change = True
        # SIMPLIFY CONSTANT POINTERS (REFRENCES)
        if str(class_name).endswith('ConstPtr'):
            class_name = class_name.replace('ConstPtr', '')
            is_some_change = True
        # SIMPLIFY POINTERS
        if str(class_name).endswith('::Ptr'):
            class_name = this.rreplace(class_name, '::Ptr', '', 1)
            is_some_change = True
        if str(class_name).endswith('Ptr'):
            class_name = this.rreplace(class_name, 'Ptr', '', 1)
            is_some_change = True
        # SIMPLIFY POINTERS
        if str(class_name).startswith('std::shared_ptr<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::shared_ptr<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('boost::shared_ptr<') and str(class_name).endswith('>'):
            class_name = class_name.replace('boost::shared_ptr<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('shared_ptr<') and str(class_name).endswith('>'):
            class_name = class_name.replace('shared_ptr<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('boost::scoped_ptr<') and str(class_name).endswith('>'):
            class_name = class_name.replace('boost::scoped_ptr<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('scoped_ptr<') and str(class_name).endswith('>'):
            class_name = class_name.replace('scoped_ptr<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        # SIMPLIFY FILTERS
        if str(class_name).startswith('tf::MessageFilter<') and str(class_name).endswith('>'):
            class_name = class_name.replace('tf::MessageFilter<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('tf2_ros::MessageFilter<') and str(class_name).endswith('>'):
            class_name = class_name.replace('tf_ros::MessageFilter<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('MessageFilter<') and str(class_name).endswith('>'):
            class_name = class_name.replace('MessageFilter<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        # SIMPLIFY VECTORS
        if str(class_name).startswith('std::vector<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::vector<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('conststd::vector<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::vector<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('vector<') and str(class_name).endswith('>'):
            class_name = class_name.replace('vector<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('std::deque<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::deque<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('deque<') and str(class_name).endswith('>'):
            class_name = class_name.replace('deque<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('std::queue<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::queue<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('queue<') and str(class_name).endswith('>'):
            class_name = class_name.replace('queue<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('std::map<') and str(class_name).endswith('>'):
            class_name = class_name.replace('std::map<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        if str(class_name).startswith('map<') and str(class_name).endswith('>'):
            class_name = class_name.replace('map<', '')
            class_name = this.rreplace(class_name, '>', '', 1)
            is_some_change = True
        # SIMPLIFY POINTER *
        if str(class_name).endswith('*'):
            class_name = this.rreplace(class_name, '*', '', 1)
            is_some_change = True
        # SIMPLIFY DANGLING ::
        if str(class_name).endswith('::'):
            class_name = this.rreplace(class_name, '::', '', 1)
            is_some_change = True
    # SANITIZED VERSION
    return class_name
