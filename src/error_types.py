#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


from enum import Enum


class AsyncErrorTypes:
    """ DEFINES TYPES OF ASYNC ERRORS
        """

    DEADLOCK_IN_CALLBACKS = 40
    SPIN_FROM_MULTIPLE_THREADS = 41  
    SPIN_FROM_CALLBACK = 42 # 1- blocking spin, 2- other spin
    MISSING_SPIN = 43 # 1- missing spin of a callback that invokes spin, 2- no spin at all
    MULTIPLE_BLOCKING_SPINS = 44
    NESTED_SPINS = 45 # 1- any spin, 2- blocking outer spin
    MULTIPLE_EXECUTORS = 46 # 1- any (same or different) types, 2- different types
    SINGLE_THREAD_FOR_PARALLEL_CALLBACKS = 47 # 1- multiple callback groups, 2- reentrant callback group
    DISABLED_CALLBACK = 48 # 1- disable callbacks of a node, 2- disabled callbacks of a callback group, 3- disabled callback

    
    ERR_TYPE_NAMES = [              # MUST BE SAME INDEX AS ERRORS LISTED ABOVE
                      'DEADLOCK_IN_CALLBACKS',
                      'SPIN_FROM_MULTIPLE_THREADS',
                      'SPIN_FROM_CALLBACK',
                      'MISSING_SPIN',
                      'MULTIPLE_BLOCKING_SPINS',
                      'NESTED_SPINS',
                      'MULTIPLE_EXECUTORS',
                      'SINGLE_THREAD_FOR_PARALLEL_CALLBACKS',
                      'DISABLED_CALLBACK'
                     ]


    SEVERITY = Enum('SEVERITY', ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'])


    def __init__(self):
        pass


    def get_err_short_discription(self, num):
        return self.ERR_TYPE_NAMES[num-40]



s = '''
DEADLOCK IN CALLBACKS
SPIN FROM MULTIPLE THREADS
SPIN FROM CALLBACK
MISSING SPIN
SINGLE THREAD FOR PARALLEL CALLBACKS
DISABLED CALLBACK
'''

