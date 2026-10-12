import os, glob

excl_paths = [
    "minimal_realtime_loop/minimal_wait_set_loop.cpp",
    "minimal_realtime_loop/minimal_polling_subscription_loop.cpp",
    "pendulum_control/src/pendulum_demo.cpp",
    "rclcpp/wait_set/",
    "rclcpp/executors/cbg_executor",
    "rclpy/executors/examples_rclpy_executors/custom_executor.py",
    "rclpy/executors/examples_rclpy_executors/custom_callback_group.py"
]

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


exec_path = ""
python_path = ""

exec_cnts = {}
# for filename in glob.iglob(outs_path + '**/**', recursive=True):
#     if filename.endswith(".out"):
#         repo = filename.replace(outs_path, "").replace(".out", "")
#         exec_cnt[repo] = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def get_exec_outs():
    crct_flag = False
    res = {}
    outs_path = "./proj_outs"
    for filename in glob.iglob(outs_path + '**/**', recursive=True):
        if filename.endswith(".out"):
            repo = filename.replace(outs_path + "/", "").replace(".out", "")
            exec_name = ""
            nodes = 0
            edges = 0
            node_cnt = 0
            exec_cnt = 0
            spin_cnt = 0
            cbgp_cnt = 0
            cbfn_cnt = 0

            ctrl_cnt = 0
            cntn_cnt = 0
            invk_cnt = 0
            nodes_crct = 0
            edges_crct = 0

            node_crct = 0
            exec_crct = 0
            spin_crct = 0
            cbgp_crct = 0
            cbfn_crct = 0

            ctrl_crct = 0
            cntn_crct = 0
            invk_crct = 0
            file = open(filename, "r")
            lines = file.readlines()
            for line in lines:
                if line.startswith("Found"):
                    # print (line)
                    if len([e for e in excl_paths if e in line]) > 0:
                        crct_flag = True
                    else:
                        crct_flag = False
                elif line.startswith("GRAPH: "):
                    split = line.replace(", ", " ").split(" ")
                    # print (split[2], split[4])
                    nodes += int(split[2])
                    edges += int(split[4])
                    if crct_flag:
                        nodes_crct += int(split[2])
                        edges_crct += int(split[4])
                elif line.startswith("TY: "):
                    split = line.replace("TY: ", "").split(" ")
                    node_cnt += int(split[0])
                    exec_cnt += int(split[1])
                    spin_cnt += int(split[2])
                    cbgp_cnt += int(split[3])
                    cbfn_cnt += int(split[4])

                    ctrl_cnt += int(split[5])
                    cntn_cnt += int(split[6])
                    invk_cnt += int(split[7])
                    if crct_flag:
                        node_crct += int(split[0])
                        exec_crct += int(split[1])
                        spin_crct += int(split[2])
                        cbgp_crct += int(split[3])
                        cbfn_crct += int(split[4])

                        ctrl_crct += int(split[5])
                        cntn_crct += int(split[6])
                        invk_crct += int(split[7])

            nodes -= nodes_crct
            edges -= edges_crct

            node_cnt -= node_crct
            exec_cnt -= exec_crct
            spin_cnt -= spin_crct
            cbgp_cnt -= cbgp_crct
            cbfn_cnt -= cbfn_crct

            ctrl_cnt -= ctrl_crct
            cntn_cnt -= cntn_crct
            invk_cnt -= invk_crct
            
            # print("%s: " % (repo))
            # print("%d %d" % (nodes, edges))
            # print("%d %d" % (nodes_crct, edges_crct))
            # print("%d %d %d %d %d %d %d %d" % (node_cnt, exec_cnt, spin_cnt, cbgp_cnt, cbfn_cnt, ctrl_cnt, cntn_cnt, invk_cnt))
            # print("%d %d %d %d %d %d %d %d" % (node_crct, exec_crct, spin_crct, cbgp_crct, cbfn_crct, ctrl_crct, cntn_crct, invk_crct))
 
            if repo in res.keys():
                cur = [nodes, edges, node_cnt, exec_cnt, spin_cnt, cbgp_cnt, cbfn_cnt, ctrl_cnt, cntn_cnt, invk_cnt]
                res[repo] = [res[repo][i] + cur[i] for i in range(10)]
            else:
                res[repo] = [nodes, edges, node_cnt, exec_cnt, spin_cnt, cbgp_cnt, cbfn_cnt, ctrl_cnt, cntn_cnt, invk_cnt]
    return res


def get_python_names():
    cwd = os.getcwd()
    stats = {}
    for r in tests_list:
        repo_name = r.replace("data/proj/", "")
        path = "./py_outs/" + repo_name + ".out"
        file = open(path, "r")
        lines = file.readlines()
        cur_script = ""
        stats_total = ""
        stats_tys = ""
        skip_flag = False
        for line in lines:
            if line.startswith("Processing"):
                stats_total = ""
                stats_tys = ""
                rel_path = line.replace("Processing ", "").replace(cwd, "").replace("/data/proj", "").replace("/" + repo_name + "/", "", 1)
                if len([e for e in excl_paths if e in rel_path]) or "launch" in rel_path or "test" in rel_path:
                    skip_flag = True
                else:
                    skip_flag = False
                    cur_script = repo_name + "/" + rel_path
            elif line.startswith("GRAPH"):
                if not skip_flag:
                    split = line.replace(", ", " ").split(" ")
                    nodes = split[2]
                    edges = split[4]
                    if int(nodes) > 0:
                        # print (cur_script + ":\n" + "%s,%s," % (nodes, edges))
                        stats_total = "%s,%s," % (nodes, edges)
            elif line.startswith("TY"):
                if not skip_flag:
                    if not line.startswith("TY: 0 0 0 0 0"):
                        # print(line.replace("TY: ", "").replace(" ", ","))
                        stats_tys = line.replace("TY: ", "").replace(" ", ",").replace("\n", "")
            if stats_total != "" and stats_tys != "":
                stats[cur_script] = stats_total.replace("\n", "") + stats_tys
    # for (k, v) in stats.items():
    #     print(k, v)

    return stats

def filter_by_out(names):
    out_path = "./proj_outs/"
    filtered = {}
    for (k, v) in names.items():
        if "//" in k:
            continue
        repo_name = k.split("/")[0]
        out_file = open(out_path + repo_name + ".out", "r")
        out_content = out_file.read()
        if not k.replace(repo_name + "/", "", 1) in out_content:
            filtered[k] = [int(value) for value in v.replace("\n", "").replace(" ", "").split(",")]
    #for (k, v) in filtered.items():
    #    print(k, v)
        
    #print (len(filtered.keys()))
    return filtered

def main():
    stats = get_exec_outs()
    #print(stats)
    names = get_python_names()
    filtered = filter_by_out(names) 
    for (k, v) in filtered.items():
        repo = k.split("/")[0]
        stats[repo] = [stats[repo][i] + filtered[k][i] for i in range(10)]
    print ("\nProject,Nodes#,Edges#,ROS_Node#,Executor#,Spin#,Callback_Group#,Callback#,0,Contain#,Invoke#\n")
    for (k, v) in stats.items():
        print (k + "," + ",".join([str(i) for i in v]))

if __name__ == '__main__':
    main()
