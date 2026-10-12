import os, glob

def get_posts_stats():
    file = open("post_outs/posts.out", "r")
    lines = file.readlines()
    res = {}
    post = ""
    for line in lines:
        if line.startswith("Processing"):
            split = line.split("/")
            post = split[-2]
            if not (post in res.keys()):
                res[post] = [0,0,0,0,0,0,0,0,0,0]
        elif line.startswith("GRAPH"):
            if post != "":
                split = line.replace(", ", " ").split(" ")
                res[post][0] += int(split[2])
                res[post][1] += int(split[4])
        elif line.startswith("TY"):
            if post != "":
                split = line.replace("TY: ", "").split(" ")
                res[post][2] += int(split[0])
                res[post][3] += int(split[1])
                res[post][4] += int(split[2])
                res[post][5] += int(split[3])
                res[post][6] += int(split[4])
                res[post][7] += int(split[5])
                res[post][8] += int(split[6])
                res[post][9] += int(split[7])
    return res
        


def main():
    res = get_posts_stats()
    print ("\nPost,Nodes#,Edges#,ROS_Node#,Executor#,Spin#,Callback_Group#,Callback#,0,Contain#,Invoke#\n")
    for (k, v) in res.items():
        print (k + "," + ",".join([str(i) for i in v]))

if __name__ == "__main__":
    main()
