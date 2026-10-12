#!/usr/bin/env python

#Copyright 2025 Purdue University.
#Copyright 2021 Purdue University, University of Virginia.
#Copyright 2018 Purdue University, University of Nebraska--Lincoln.
#Copyright (c) 2016, University of Nebraska NIMBUS LAB  John-Paul Ore jore@cse.unl.edu
#All rights reserved.


import click
import os
import sys
from distutils import spawn
from subprocess import Popen, PIPE
from time import gmtime, strftime, mktime
#from datetime import datetime

# IF INSTALLED GLOBALLY, SET VALUE TO 'cppcheck'
CPPCHECK_EXECUTABLE = 'cppcheck'

# support testing setup as well as testing on any other machine
# set it to False to enable testing on any machine other than testing setup
TESTING_SETUP= True

if TESTING_SETUP:
    CPPCHECK_EXECUTABLE = '/cppcheck-1.90/cppcheck'



@click.command()
@click.argument('target')
@click.option('--launch_file', default='', help='launch file to start the application')
@click.option('--display_graph', default=True, help='display the generated graph')
@click.option('--analyze_separate', default=True, help='analyze each executable separately')
@click.option('--lockset', is_flag=True, help='run only the lightweight C++ lockset analysis')
@click.option('--lockset-output', default='', help='lockset JSON path (default: TARGET/lockset.json)')
def main(target, launch_file, display_graph, analyze_separate, lockset, lockset_output):

    if lockset:
        from lockset_analysis import analyze_project
        output = lockset_output or os.path.join(
            target if os.path.isdir(target) else os.path.dirname(os.path.abspath(target)),
            'lockset.json')
        analyze_project(target, output, CPPCHECK_EXECUTABLE if os.path.exists(CPPCHECK_EXECUTABLE) else 'cppcheck')
        print('Created %s' % output)
        return

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # TEST FOR CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if not os.path.exists(CPPCHECK_EXECUTABLE):
        if not spawn.find_executable('cppcheck'):
            print('Could not find required program cppcheck: %s' % CPPCHECK_EXECUTABLE) 
            sys.exit(1)

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    # RUN CPPCHECK
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if not os.path.exists(target):
        print( 'target does not exist: %s' % target)
        sys.exit(1) 

    if launch_file and not os.path.exists(launch_file):
        print( 'launch file does not exist: %s' % launch_file)
        sys.exit(1)

    print( 'Processing %s' % target)
    print( 'Attempting to run cppcheck...')

    FORCE_CPPCHECK = False

    original_directory = os.getcwd()
    target_dir = target if os.path.isdir(target) else os.path.dirname(target)
    print( 'Changing directory to %s' % target_dir)
    os.chdir(target_dir)

    dump_filename = 'dump.temp' if os.path.isdir(target) else os.path.basename(target) + '.dump'

    if FORCE_CPPCHECK or not os.path.exists(dump_filename):
        dirlist = get_header_dirlist(target_dir)

        include_str = ''
        for d in dirlist:
            include_str += ' -I ' + d
   
        args = [CPPCHECK_EXECUTABLE, '--dump', include_str, target]
        print( "Running cppcheck command: %s" % ' '.join(args))
        cppcheck_process = Popen(' '.join(args),  shell=True, stdout=PIPE, stderr=PIPE)
        out, err = cppcheck_process.communicate()
        print(out)
        out_str = out.decode("utf-8")
        if cppcheck_process.returncode != 0 and \
                not out_str.startswith('cppcheck: error: could not find or open any of the paths given'):
            print( 'cppcheck appears to have failed..exiting with return code %d' % cppcheck_process.returncode)
            # sys.exit(1)
        open(dump_filename, 'a').close()
        print( "Created cppcheck 'dump' file %s" % dump_filename)

    os.chdir(original_directory)

    #create output directory if does not exist
    if not os.path.exists('output'):
        os.mkdir('output')

    t_start = gmtime()
    print("STARTING... %s " % strftime("%Y-%m-%d %H:%M:%S", t_start))
    #t_s = datetime.now()
    if not analyze_separate:
        analyze(target, launch_file, display_graph)
    else:
        analyze_per_executable(target, launch_file, display_graph)

    t_end = gmtime()
    print("ENDING... %s (%s sec) " % (strftime("%Y-%m-%d %H:%M:%S", t_end), mktime(t_end)-mktime(t_start)))
    #t_e = datetime.now()
    #print("(%s) (%s ms)" % ((t_e-t_s), (t_e-t_s).total_seconds()*1000))
    print('*'*80)


def analyze_per_executable(target, launch_file, display_graph):
    from graph_collector import GraphCollector
    gr_collector = GraphCollector()

    errors_file = 'errors.txt'
    display_name = 'output/cbexec_graph.png' if display_graph else ''

    if display_graph:
        original_directory = os.getcwd()

        if not TESTING_SETUP:       
            out_dirname = 'output'
            if not os.path.exists(out_dirname):
                os.mkdir(out_dirname)
            errors_file = out_dirname + '/' + errors_file

            names = target.split('/')
            if len(names) > 1:
                os.chdir(out_dirname)
                if not os.path.exists(names[-2]):
                    os.mkdir(names[-2])
                display_name = out_dirname + '/' + names[-2]
                out_dirname = names[-2]
        else:
            out_dirname = 'outs'
            if not os.path.exists(out_dirname):
                os.mkdir(out_dirname)
            errors_file = out_dirname + '/' + errors_file

            display_name = out_dirname

        if os.path.isdir(target):
            os.chdir(out_dirname)

            if target.endswith('/'):
                target = target[:-1]
            proj_dirname = 'graph_' + os.path.basename(target)
            if not os.path.exists(proj_dirname):
                os.mkdir(proj_dirname)
            display_name = display_name + '/' + proj_dirname
            
        else:
            display_name = display_name + '/graph_' + (os.path.basename(target).replace('.', '_')) + '.png'
                
        os.chdir(original_directory)

    # ANALYZE
    gr_collector.construct_check_and_display_graph(target, launch_file, display_name, errors_file)


def analyze(target, launch_file, display_graph):
    #try:   
    from graph_collector import GraphCollector
    gr_collector = GraphCollector()

    gr_collector.construct_graph(target, launch_file)

    errors_to_file = True if (not TESTING_SETUP) else False
    gr_collector.check_graph(errors_to_file)

    print ("target: ", target) ##TODO why do we need it?
    if display_graph:
        if not TESTING_SETUP:
            names = target.split('/')
            if len(names) > 1:
                name = 'output/' + names[-2]
            else:
                name = None
        else: # TESTING_SETUP
            print ("out:", target)
            relative_path = target
            names = relative_path.split('/')
            graph_dir = "/outs/graph_" + names[-2] + "/"
            if not os.path.exists(graph_dir):
                os.mkdir(graph_dir)
                print (graph_dir)
            if target.endswith(".py") or target.endswith(".cpp"):
                name = graph_dir + (relative_path.replace(".py", "_py").replace(".cpp", "_cpp").replace("/", "_")) + ".png"
            else:
                name = graph_dir + names[0] + ".png"
            print("graph name:", name)

        gr_collector.display_graph(name)
    #except Exception as e:
        #print("EXCEPTION: %s" % str(e))
        #raise
    


def get_header_dirlist(topdir):
    dirset = set()
    templist = []

    for root, dirs, files in os.walk(topdir):
        for f in files:
            if any(f.endswith(s) for s in ['.h', '.hpp', '.hh']):
                templist.append(root)
                break

    rootp = topdir.rstrip('/')

    for d in templist:
        dirset.add(d)

        c = d
        while (c != rootp):
            p = os.path.abspath(os.path.join(c, os.pardir))
            dirset.add(p)
            c = p

    dirset.add(rootp)

    return list(dirset)


if __name__ == '__main__':
    main()
