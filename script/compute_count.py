import os

POST_ERR_FILE = 'post_outs/errors.txt'
PROJ_ERR_FILE = 'proj_outs/errors.txt'
PY_ERR_FILE = 'py_outs/errors.txt'

excl_paths = [
    "minimal_realtime_loop/minimal_wait_set_loop.cpp",
    "minimal_realtime_loop/minimal_polling_subscription_loop.cpp",
    "pendulum_control/src/pendulum_demo.cpp",
    "rclcpp/wait_set/",
    "rclcpp/executors/cbg_executor",
    "rclpy/executors/examples_rclpy_executors/custom_executor.py",
    "rclpy/executors/examples_rclpy_executors/custom_callback_group.py"
]

msg_type_list = ['DIC',
                 'OTFPC_reentrant',
                 'OTFPC_multiple',
                 'ME_any',
                 'ME_different',
                 'SFMT',
                 'SFC_other',
                 'SFC_blocking',
                 'MS_no_outer',
                 'MS_no',
                 'MBS',
                 'NS_blocking',
                 'NS_any',
                 'DC_group',
                 'DC_cb']

post_cnt = {}
proj_cnt = {}


def init_post_cnt_dict():
    for msg_type in msg_type_list:
        post_cnt[msg_type] = {'cnt': 0, 'source_id_set': set(), 'msg_list': None}
    

def add_post_cnt_entry(msg_type, msg_source_id, msg=None):
    msg_dict = post_cnt[msg_type]
    msg_dict['cnt'] += 1
    msg_dict['source_id_set'].add(msg_source_id)
    
    
def get_post_cnt_msg_details(msg_type):
    msg_dict = post_cnt[msg_type]
    return (msg_dict['cnt'], len(msg_dict['source_id_set']))
    

def init_proj_cnt_dict():
    for msg_type in msg_type_list:
        proj_cnt[msg_type] = {'cnt': 0, 'source_id_set': set(), 'msg_list': []}
    

def add_proj_cnt_entry(msg_type, msg_source_id, msg):
    msg_dict = proj_cnt[msg_type]
    msg_dict['cnt'] += 1
    msg_dict['source_id_set'].add(msg_source_id)
    msg_dict['msg_list'].append(msg)
    
    
def add_py_cnt_entry(msg_type, msg_source_id, msg):
    msg_dict = proj_cnt[msg_type]
    if msg_source_id in msg_dict['source_id_set'] and msg in msg_dict['msg_list']:
        return
    #else
    msg_dict['cnt'] += 1
    msg_dict['source_id_set'].add(msg_source_id)
    msg_dict['msg_list'].append(msg)
                

def get_proj_cnt_msg_details(msg_type):
    msg_dict = proj_cnt[msg_type]
    return (msg_dict['cnt'], len(msg_dict['source_id_set']))
    
    
def get_msg_details(msg_type):
    post_msg_n, post_source_n = get_post_cnt_msg_details(msg_type)
    proj_msg_n, proj_source_n = get_proj_cnt_msg_details(msg_type)
    return (post_msg_n + proj_msg_n, post_msg_n, proj_msg_n, post_source_n, proj_source_n)


def get_msg_type(parts):
    if parts[0] == 'DEADLOCK_IN_CALLBACKS':
        return 'DIC'
    elif parts[0] == 'SINGLE_THREAD_FOR_PARALLEL_CALLBACKS':
        if parts[1].startswith(" Warning#: Reentrant"):
            return 'OTFPC_reentrant'
        else:
            return 'OTFPC_multiple'
    elif parts[0] == 'MULTIPLE_EXECUTORS':
        if parts[1].endswith("at a time."):
            return 'ME_any'
        else:
            return 'ME_different'
    elif parts[0] == 'SPIN_FROM_MULTIPLE_THREADS':
        return 'SFMT'
    elif parts[0] == 'SPIN_FROM_CALLBACK':
        if parts[1].startswith(" Warning#:"):
            return 'SFC_other'
        else:
            return 'SFC_blocking'
    elif parts[0] == 'MISSING_SPIN':
        if parts[1].startswith(" Error#: Missing"):
            return 'MS_no_outer'
        else:
            return 'MS_no'
    elif parts[0] == 'MULTIPLE_BLOCKING_SPINS':
        return 'MBS'
    elif parts[0] == 'NESTED_SPINS':
        if 'blocking' in parts[1]:
            return 'NS_blocking'
        else:
            return 'NS_any'
    elif parts[0] == 'DISABLED_CALLBACK':
        if parts[1].startswith(" Error#: all"):
            return 'DC_group'
        else:
            return 'DC_cb'
    else:
        return ''
        


def main():
    init_post_cnt_dict()
    init_proj_cnt_dict()
    
    if os.path.exists(POST_ERR_FILE):
        post_f = open(POST_ERR_FILE, 'r')
        source_id = None
        f_group = False
        
        for line in post_f:
            if line.rstrip() == "":
                continue
            elif line.rstrip() == "-":
                f_group = not f_group
                continue    
                
            parts = line.rstrip().split(';')
            if len(parts) == 1:
                if not f_group:
                    parts = line.rstrip().split('/')
                    source_id = parts[2]
            elif len(parts) == 2:
                msg_type = get_msg_type(parts)
                if msg_type:
                    add_post_cnt_entry(msg_type, source_id)
            
        post_f.close()
        
    if os.path.exists(PROJ_ERR_FILE):
        proj_f = open(PROJ_ERR_FILE, 'r')
        source_id = None
        f_group = False
        
        for line in proj_f:
            if line.rstrip() == "":
                continue
            elif line.rstrip() == "-":
                f_group = not f_group
                if f_group: #start of filegroup - reset the skip_msg flag
                    skip_msg = False
                continue
            
            parts = line.rstrip().split(';')
            if len(parts) == 1:
                if f_group:
                    if not skip_msg:
                        skip_msg = True if len([e for e in excl_paths if e in parts[0]]) > 0 else False
                else:
                    parts = line.rstrip().split('/')
                    source_id = parts[2]
            elif len(parts) == 2:
                msg_type = get_msg_type(parts)
                if msg_type:
                    if not skip_msg:
                        add_proj_cnt_entry(msg_type, source_id, (source_id,line))
            
        proj_f.close()
        
    if os.path.exists(PY_ERR_FILE):
        py_f = open(PY_ERR_FILE, 'r')
        source_id = None
        
        for line in py_f:
            if line.rstrip() == "":
                continue
            parts = line.rstrip().split(';')
            if len(parts) == 1:
                if parts[0].endswith('.py'):
                    skip_msg = True if len([e for e in excl_paths if e in parts[0]]) > 0 else False
                else:
                    parts = line.rstrip().split('/')
                    source_id = parts[2]
            elif len(parts) == 2:
                msg_type = get_msg_type(parts)
                if msg_type:
                    if not skip_msg:
                        add_py_cnt_entry(msg_type, source_id, (source_id,line))
            
        py_f.close()
        
        
    
    print("DIC (e1, e2, e3)                = %d (%d, %d) in (%d, %d)" % get_msg_details('DIC'))
    print("OTFPC (w4: multiple)            = %d (%d, %d) in (%d, %d)" % get_msg_details('OTFPC_multiple'))
    print("OTFPC (w5: reentrant)           = %d (%d, %d) in (%d, %d)" % get_msg_details('OTFPC_reentrant'))
    print("ME (e6: different)              = %d (%d, %d) in (%d, %d)" % get_msg_details('ME_different'))
    print("ME (e7: any type)               = %d (%d, %d) in (%d, %d)" % get_msg_details('ME_any'))
    print("SFMT (e8)                       = %d (%d, %d) in (%d, %d)" % get_msg_details('SFMT'))
    print("SFC (e9: blocking spin)         = %d (%d, %d) in (%d, %d)" % get_msg_details('SFC_blocking'))
    print("SFC (w10: other timed spins)    = %d (%d, %d) in (%d, %d)" % get_msg_details('SFC_other'))
    print("MS (e11: no outer initial spin) = %d (%d, %d) in (%d, %d)" % get_msg_details('MS_no_outer'))
    print("MS (e12: no spin at all)        = %d (%d, %d) in (%d, %d)" % get_msg_details('MS_no'))
    print("MBS (e13)                       = %d (%d, %d) in (%d, %d)" % get_msg_details('MBS'))
    print("NS (e14: any spins)             = %d (%d, %d) in (%d, %d)" % get_msg_details('NS_any'))
    print("NS (e15: blocking outer spin)   = %d (%d, %d) in (%d, %d)" % get_msg_details('NS_blocking'))
    print("DC (e16: for group)             = %d (%d, %d) in (%d, %d)" % get_msg_details('DC_group'))
    print("DC (e17: callback)              = %d (%d, %d) in (%d, %d)" % get_msg_details('DC_cb'))
        


if __name__ == "__main__":
    main()
