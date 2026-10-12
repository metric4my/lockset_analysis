import os, glob
from subprocess import Popen
import shutil

tests_list = [
'data/proj/whisper_ros',
'data/proj/ros2_RobotSimulation',
'data/proj/cdfr',
'data/proj/raspimouse_ros2_examples',
'data/proj/dbw_ros',
'data/proj/iiwa_ros2',
'data/proj/Plankton',
'data/proj/octomap_mapping',
'data/proj/foros',
'data/proj/aplotter_ros2',
'data/proj/ros2-realtime-examples',
'data/proj/ros2_computer_vision',
'data/proj/audio_common',
'data/proj/cyberdog_ros2',
'data/proj/UNav-Sim',
'data/proj/stcamera_ros2',
'data/proj/realsense_hardware_interface',
'data/proj/QUTMS_Driverless',
'data/proj/gazebo_ros2_control',
'data/proj/demos',
'data/proj/examples',
'data/proj/crane_x7_ros',
'data/proj/robot_calibration',
'data/proj/marti_common',
'data/proj/TurtleBot3',
'data/proj/ublox',
'data/proj/webots_ros2',
'data/proj/quad_ws',
'data/proj/ros2_shm_msgs',
'data/proj/teleop_tools'
]

def main():
    working_directory = os.getcwd()
    
    output_dir = 'outs'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    
    errors_file = output_dir + '/errors.txt'
    with open(errors_file, 'w') as f:
        f.write("\n")
    
    for test in tests_list:
    
        test_path = os.path.join(working_directory, test)
        if not os.path.exists(test_path):
            continue

        with open(errors_file, 'a') as f:
            f.write("\n\n%s\n" % test)
            
        out_file = output_dir + '/' + test.split('/')[-1] + '.out'
        out_log = open(out_file, 'a')
    
        for filename in glob.iglob(test_path + '**/**', recursive=True):
            if filename.endswith(".py"): 
                with open(errors_file, 'a') as f:
                    f.write("\n%s\n" % filename.replace(test_path, ""))
                args = ['python3', 'src/roscbexec.py', filename]
                roscbexec_process = Popen(' '.join(args), shell=True, stdout=out_log, stderr=out_log)
                roscbexec_process.communicate()
                if roscbexec_process.returncode != 0:
                    print("roscbexec appears to have failed for %s..exiting with return code %d\n" % (test, roscbexec_process.returncode))
            
        out_log.close()
        
    if os.path.exists("py_outs"):
        shutil.rmtree("py_outs")
    shutil.move("outs", "py_outs")

if __name__ == "__main__":
    main()
