import numpy as np

def createZigZagHalf(pose, length):
    poses = []
    app_pose = pose
    disc = np.sqrt(length)/10000.
    add_pos = [disc, disc, 0, 0,0,0]
    add_neg = [disc, -disc, 0, 0,0,0]
    for i in range(length):
        if(i<length/2):
            app_pose = app_pose+add_pos
        elif(i>=length/2):
            app_pose = app_pose+add_neg
        poses.append(app_pose)
    return poses

def createZigZagPattern(pose, length, num):
    poses = createZigZagHalf(pose, length)
    for i in range(num-1):
        add_poses = createZigZagHalf(poses[-1], length)
        for j in add_poses:
            poses.append(j)
    return poses

