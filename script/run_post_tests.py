import os
from subprocess import Popen
import shutil


tests_list = [
'data/post/42188/path_planning.cpp',
'data/post/42188/path_planning_solution.cpp',
'data/post/43088/time_synchronizer.cpp',
'data/post/240925/goforward_bumpandturn.py',
'data/post/241861/my_node.cpp',
'data/post/244975/image_converter.cpp',
'data/post/244975/image_converter_updated.cpp',
'data/post/259765/sub_with_timer',
'data/post/297204/multi_spin_node.cpp',
'data/post/302037/test.py',
'data/post/302037/test_service.py',
'data/post/302037/test_service_2.py',
'data/post/302037/sub_calling_service_solution.py',
'data/post/304085/demonode.py',
'data/post/305847/my_node.cpp',
'data/post/328378/main_window.cpp',
'data/post/330706/listener.cpp',
'data/post/334760/kuka.cpp',
'data/post/342302/service_call_pkg',
'data/post/342302/service_call_pkg_solution',
'data/post/345286/mynode.py',
'data/post/345286/mynode_solution.py',
'data/post/345286/wizardnode.py',
'data/post/358343/simplenode.py',
'data/post/358343/simplenode_solution.py',
'data/post/361637/exact_time.cpp',
'data/post/361637/exact_time_solution.cpp',
'data/post/364975/timers_4',
'data/post/364975/timers_3_solution',
'data/post/366440/temp_sync.cpp',
'data/post/366440/temp_sync_solution.cpp',
'data/post/371171/Escritor.py',
'data/post/373169/srv.py',
'data/post/373169/srv_solution_bug.py',
'data/post/373169/srv_solution.py',
'data/post/377848/py3_multiple_spin_threads.py',
'data/post/379116/RobotControl.py',
'data/post/379259/multi_subscriber_1.cpp',
'data/post/379259/multi_subscriber_2.cpp',
'data/post/379999/trigger_transition_server.py',
'data/post/395870/zones_sub.py',
'data/post/395870/zones_sub_solution.py',
'data/post/406915/client_handler.cpp',
'data/post/406915/client_handler_solution.cpp',
'data/post/57893546/controller_node_test.cpp',
'data/post/75822409/monitor_node.py',
'data/post/75822409/monitor_node_solution_bug.py',
'data/post/75822409/monitor_node_solution.py',
'data/post/1018/minimal_pkg',
'data/post/1018/minimal_pkg_solution'
]


def main():
    working_directory = os.getcwd()
    
    output_dir = 'outs'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
        
    errors_file = output_dir + '/errors.txt'
    with open(errors_file, 'w') as f:
        f.write("\n")
        
    out_file = output_dir + '/posts.out'
    out_log = open(out_file, 'a')
    
    for test in tests_list:
    
        with open(errors_file, 'a') as f:
            f.write("\n\n%s\n" % test)
    
        test_path = os.path.join(working_directory, test)
        args = ['python3', 'src/roscbexec.py', test_path]
        roscbexec_process = Popen(' '.join(args),  shell=True, stdout=out_log, stderr=out_log)
        roscbexec_process.communicate()
        if roscbexec_process.returncode != 0:
            print("roscbexec appears to have failed for %s..exiting with return code %d\n" % (test, roscbexec_process.returncode))
            continue
            
    out_log.close()
    
    if os.path.exists("post_outs"):
        shutil.rmtree("post_outs")
    shutil.move("outs", "post_outs")


if __name__ == "__main__":
    main()
    

