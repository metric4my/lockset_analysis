#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


from error_types import AsyncErrorTypes


class AsyncError:

    def __init__(self):
        self.ERROR_TYPE = None 
        self.SEVERITY = None
        self.sub_type = None
        self.values = None
        self.var_name = ''
        self.file_name = ''
        self.linenr = 0


    def get_error_desc(self):
        return AsyncErrorTypes().get_err_short_discription(self.ERROR_TYPE)
