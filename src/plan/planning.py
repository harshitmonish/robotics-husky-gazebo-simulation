#!/usr/bin/env python
# You may only use the functions imported below
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import cv2
import rospy
import math
from collections import defaultdict
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, TwistStamped
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import heapq
from tf.transformations import euler_from_quaternion
import matplotlib
#matplotlib.use('Qt5Agg')
#global defines
initial_x = None
initial_y = None
goal_x = None
goal_y = None
husky_name = "husky::base_link"
pose = None
initial_pos_set_flag = False
last_pos = None
odom_msg = None
map_=[
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
[1,0,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
[1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
[1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
[1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
[1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,1],
[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]]

map_copy = map_.copy()

map_ = np.asarray(map_).reshape((42, 42))

#converting from global map to grid
def world_to_grid(cord):
    x, y = cord[0], cord[1]
    x_, y_ = 21 - y, 21 + x
    return [x_, y_]

#converting from grid to global map
def grid_to_world(cord):
    x = cord[0]
    y = cord[1]
    x_, y_ = y - 21, 21 - x
    return [x_, y_]

#function to calculate euclidean distance
def euc(p1, p2):
    e = 1
    return e*(math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2))

#function to check if new coordinates are within the map
def is_safe(cord, arr):
    if 0 <= cord[0] < arr.shape[0] and 0 <= cord[1] < arr.shape[1] and arr[int(cord[0])][int(cord[1])] != 1:
        return True
    return False

# A* algorithm
def a_star_func(arr, start_pos, goal_pos):
    #defining 4 neighbours to search
    adj = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    global last_pos

    #initialize parameters
    visited = set()
    last_pos = {}
    heap = []
    mp1 = {start_pos:0}
    mp2 = {start_pos:euc(start_pos, goal_pos)}
    heapq.heappush(heap, (mp2[start_pos], start_pos))

    # main loop
    while heap:
        #fetching 1st node
        curr = heapq.heappop(heap)[1]

        #if goal is achieved break
        if curr == goal_pos:
            res = []
            while curr in last_pos:
                res.append((curr, mp1[curr]))
                curr = last_pos[curr]
            return res

        #else add to visited set
        visited.add(curr)

        #explore all 4 neighbours
        for i, j in adj:
            new_pos = curr[0]+i, curr[1] + j
            dist = mp1[curr] + euc(curr, new_pos)
            if not is_safe(new_pos, arr):
                continue
            # already visited, continue
            if new_pos in visited and dist >= mp1.get(new_pos, 0):
                continue

            if dist < mp1.get(new_pos, 0) or new_pos not in [hp[i] for hp in heap]:
                #add this node 
                last_pos[new_pos] = curr
                mp1[new_pos] = dist
                mp2[new_pos] = dist + euc(new_pos, goal_pos)
                heapq.heappush(heap, (mp2[new_pos], new_pos))
    return []

def callback_ls(msg):
    global pose
    global initial_pos_set_flag
    global initial_x
    global initial_y
    idx = msg.name.index(husky_name)
    x, y = round(np.float64(msg.pose[idx].position.x), 0), round(np.float64(msg.pose[idx].position.y), 0)
    if not initial_pos_set_flag:
        initial_x = x
        initial_y = y
    pose = (x, y)
    initial_pos_set_flag = True

def callback_odom(msg):
    global odom_msg
    #print("odom callback")
    odom_msg = msg

def main():
    rospy.init_node('lab5_planning')
    global goal_x
    global goal_y
    global initial_x
    global initial_y

    #subscribing to gazebo/link_states
    sub1 = rospy.Subscriber('/gazebo/link_states', LinkStates, callback_ls, queue_size=100)
    #fetching initial and final poition from launch file
    #initial_x = np.float64(rospy.get_param("/lab5/initial_x"))
    #initial_y = np.float64(rospy.get_param("/lab5/initial_y"))
    goal_x = np.float64(rospy.get_param("/lab5/goal_x"))
    goal_y = np.float64(rospy.get_param("/lab5/goal_y"))

    r = rospy.Rate(5)
    if initial_pos_set_flag == False:
        while initial_pos_set_flag == False:
            print("Waiting for the callback from /gazebo/link_states")
    
    print(initial_x, initial_y)
    init_w2gd = world_to_grid([initial_x, initial_y])
    goal_w2gd = world_to_grid([goal_x, goal_y])
    print("Initial grid coordinates are: ")
    print(init_w2gd)
    print("Goal grid coordinates are : ")
    print(goal_w2gd)

    #if initial pos is wall return 
    if init_w2gd == 1:
        print("INVALID starting point")
        return -1
    if goal_w2gd == 1:
        print("INVALID Goal position")

    print("Evaluating the planning path")
    mp_temp = np.asarray(map_, dtype=object).reshape((42, 42))
    # evaluating optimal path
    path = a_star_func(map_, (init_w2gd[0], init_w2gd[1]), (goal_w2gd[0], goal_w2gd[1]))
    n = len(path)
    path_plot_x = []
    path_plot_y = []
    if n == 0:
        print("Error No Path Found ! ")
        exit(0)
    else:
        print("\n \n Sequence of Node indexes is : ")
        for i in range(n-1, -1, -1):
            print(path[i][0], " dist : " , path[i][1] ,end= " -> ")
            path_plot_x.append(path[i][0][0])
            path_plot_y.append(path[i][0][1])
        print("\n \n Absolute position is : ")
        for i in range(n-1, -1, -1):
            print(grid_to_world(path[i][0]), end=" -> ")
        print()

    print("Plotting in matplotlib")
    labels = {1:'obstacle', 0:'open_path'}
    plt.plot(path_plot_x, path_plot_y, label="planned path", color="red")
    plt.imshow(map_)
    cmap = {1:[1.0, 1.0, 0.1, 1], 0:[0.1, 1.0, 0.1, 1]}
    #plt.colorbar()
    patches = [mpatches.Patch(color=cmap[i], label=labels[i]) for i in cmap]
    plt.legend(handles=patches)
    plt.savefig("./map_plot.png")
    img = cv2.imread("./map_plot.png", cv2.IMREAD_COLOR)
    cv2.imshow('image', img)
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
    #plt.show()

    #converting to world coordinates resp
    wpath = []
    for i in path:
        wpath.append(grid_to_world(i[0]))

    global count
    global newp_x
    global newp_y
    global pose
    global temp
    global odom_msg

    count = 0
    newp_x = wpath[-1-count][0]
    newp_y = wpath[-1-count][1]

    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    sub2 = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, callback_odom, queue_size=20)
    print("Start running")
    #rospy.spin()
    while not rospy.is_shutdown():
        #try:
            sub2 = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, callback_odom, queue_size=20)
            if odom_msg != None:
                pos = odom_msg.pose.pose.position
                newp_x = wpath[-1-count][0]
                newp_y = wpath[-1-count][1]
                if newp_x == wpath[0][0]:
                    print("Reached the Destination! exiting")
                    exit(0)
                dist = math.sqrt((pos.x - newp_x)**2 + (pos.y - newp_y)**2)
                cmd_vel = Twist()

                ang = odom_msg.pose.pose.orientation
                (x_, y_, ang_) = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
                ang_del = math.atan2(newp_y - pos.y, newp_x - pos.x)
                ang_del1 = ang_del - ang_

                cmd_vel.angular.z = ang_del1

                if abs(ang_del1) < 0.02:
                    cmd_vel.angular.z = 0
                    cmd_vel.linear.x = 0.5
                else:
                    cmd_vel.angular.z = ang_del1
                    cmd_vel.linear.z = 0
                pub.publish(cmd_vel)

                if dist < 0.30:
                    count+=1
            #r.sleep()
        #except:
        #    pass
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


