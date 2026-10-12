#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


import os
import fnmatch


def find_files(directory, pattern):
    if os.path.isfile(directory):
        yield directory

    # THIS IS A DIRECTORY
    for root, dirs, files in os.walk(directory):
        root = os.path.abspath(root)
        for basename in files:
            if fnmatch.fnmatch(basename.lower(), pattern):
                filename = os.path.join(root, basename)
                yield filename
