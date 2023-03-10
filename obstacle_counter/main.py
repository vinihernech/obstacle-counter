import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from laser_geometry import LaserProjection
from math import  cos, sin
import sensor_msgs_py.point_cloud2 as pc2
import sensor_msgs.msg as sensor_msgs
import numpy as np
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from sklearn.cluster import DBSCAN
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial.distance import cdist
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import random
from threading import Thread
import sys

#Global Variables
fig,axis = plt.subplots(1,1)
plt.show(block=False)
lp = LaserProjection()
global_scan = np.array([1.5,0.5])
global_robot_pose = np.array([0,0])
global_cluster = list()


class Perception_Node(Node):
    def __init__(self):
        super().__init__("ExampleNode")
        self.image_sub = Subscriber(self, Odometry, "/odom")
        self.plc_sub = Subscriber(self, sensor_msgs.LaserScan, "/scan")
        queue_size = 30
        self.function = run

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.plc_sub],
            queue_size,
            0.1,  
        )
        self.ts.registerCallback(self.callback)
    def callback(self, msg_odom, msg_scan):
        self.function(msg_odom,msg_scan)


def send_goal(navigator):
    k = random.randint(0,3)
    if k==0:
        x,y= -2.0,0.0
    elif k ==1:
        x,y= 2.0,0.0
    elif k ==2:
        x,y= 0.0,2.0
    else:   
        x,y= 0.0,-2.0
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)

def run(msg_odom,msg_laser):
    # convert the message of type LaserScan to a PointCloud2
    global global_scan
    global global_robot_pose
    global global_cluster
    local_scan = list()
    robot_pose = list()
    clusters = list()
    global_robot_pose = np.array([msg_odom.pose.pose.position.x,msg_odom.pose.pose.position.y])
    _,_,yaw = euler_from_quaternion(msg_odom.pose.pose.orientation)
    robot_pose.append([msg_odom.pose.pose.position.x,msg_odom.pose.pose.position.y, yaw])  
    pc2_msg = lp.projectLaser(msg_laser)
    point_generator = pc2.read_points(pc2_msg)
    final_list = list()
    for point in point_generator: 
        a = (point[0]*cos(robot_pose[0][2]) - point[1]*sin(robot_pose[0][2])) + (robot_pose[0][0] - 0.032)
        b = (point[0]*sin(robot_pose[0][2]) + point[1]*cos(robot_pose[0][2])) + robot_pose[0][1]
        final_list.append([a,b])          
    local_scan.append(final_list)
    current_scan = np.unique(np.round(np.array(local_scan[0]),decimals= 1),axis=0)
    if global_scan.shape == (2,):
        global_scan = np.round(np.array(local_scan[0]), decimals= 1)
    global_scan = np.unique(np.round(np.vstack([global_scan, current_scan]),decimals= 1), axis=0)
    db = DBSCAN(eps=0.13, min_samples=3).fit(global_scan)
    labels =db.labels_ 
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)

    core_samples_mask[db.core_sample_indices_] = True
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    i=0
    for k, col in zip(unique_labels, colors):
        if k == -1:
            col = [0, 0, 0, 1]
        class_member_mask = labels == k
        xy = global_scan[class_member_mask & core_samples_mask]

        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "o",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=15,
        )
        i=i+1
        clusters.append(xy)  
        if i == n_clusters_:
            global_cluster = clusters
        plt.title(f"Estimated number of obstacles: {n_clusters_}")
        fig.canvas.draw()
        fig.canvas.flush_events()
        if len(xy) >=340:
            print("All obstacles were detected")
            for i in range(len(global_cluster)):
                if len(global_cluster[i])<= 40:
                    try:
                        a, b = centeroidnp(global_cluster[i])
                        with open('obstacle.txt', 'a') as f:
                            f.write(f'\n Obstacle {i} position:{a,b}')
                    except:
                        pass
            plt.savefig('obstacles.png')
            print("Storing the results.")
            sys.exit()


def centeroidnp(arr):
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x/length, sum_y/length


def euler_from_quaternion(quaternion):

    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def main(args=None):

    rclpy.init(args=None)
    main_subscriber = Perception_Node()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = float(global_robot_pose[0])
    initial_pose.pose.position.y = float(global_robot_pose[1])
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    th = Thread(target=send_goal(navigator))
    th.start()
    th.join()
    i = 0
    print("Starting ...")
    while rclpy.ok():
        i = i+1
        rclpy.spin_once(main_subscriber)
        if i == 130:
            th= Thread(target=send_goal(navigator))
            th.start()
            th.join()
            i=0

if __name__ == '__main__':
    main()
