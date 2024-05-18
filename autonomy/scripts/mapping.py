#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg._ModelStates import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from autonomy_config import CAMERA_POSITION_OFFSET, ROUNDING_COEF, ROVER_MODEL_NAME
import numpy as np
from typing import Set, Tuple, List
from scipy.spatial import ConvexHull
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from queue import PriorityQueue, Queue

class Block:
    def __init__(self):
        self.avg = None  
        self.n = 0  
        self.gradient = None
        self.on_path = False

class PointCloudTracker:
    # 2D Array representing the map. Each entry is linked to a 0.1m square
    # The square (0t0.1, 0t0.1) is [150][150]
    # X = (floor(x) + 150) +  floor(dec(x)*10) 
    # Entry = 0(Green): This block is known to be obstacless
    # Entry = 1(Yellow): We dont know if there is or not an obstacle
    # Entry = 2, 3, 4, 5, 6, ...(Red): This block is known to be obstaclefull. If entry is 3, you know
    # the block is part of obstacle 3. This means that there are other red blocks around it that are part of obstacle 3 
    # You might need to perform obstacle merging.
    n1 = 300
    n2 = 300
    map = np.empty((n1, n2), dtype=Block)
    
    surr_8 = [[-1, 1], [-1, 0], [-1, -1], [0, 1], [1, 1], [0, -1], [1, -1], [1, 0]]
    surr_40 = [[0,7],[1,7],[2,7],[3,7],[4,7],[4,6],[5,6],[6,6],[6,5],[6,4],[7,4],[7,3],
               [7,2],[7,1],[7,0],[7,-1],[7,-2],[7,-3],[7,-4],[6,-4],[6,-5],[6,-6],[5,-6],
               [4,-6],[4,-7],[3,-7],[2,-7],[1,-7],[0,-7],[-1,7],[-2,7],[-3,7],[-4,7],[-4,6],
               [-5,6],[-6,6],[-6,5],[-6,4],[-7,4],[-7,3],[-7,2],[-7,1],[-7,0],[-7,-1],[-7,-2],
               [-7,-3],[-7,-4],[-6,-4],[-6,-5],[-6,-6],[-5,-6],[-4,-6],[-4,-7],[-3,-7],[-2,-7],[-1,-7]]
    
    # Current Position
    pos = Point()
    pos.x = 0
    pos.y = 0
    pos.z = 0

    # Current Orientation
    ori = Quaternion()

    # Landmark arrival tolerance
    tol_d = 0.1

    # Contains open nodes, and their -F score to keep order
    openset = PriorityQueue()

    # Node table. Contains all known nodes as "key" and their G score, and previous node as "values"
    node_table = dict()

    # Entire Landmarks Left
    landmarks = [[7,0]]
    
    # Current full path
    cur_p = None

    # Frame ID
    FIXED_FRAME_ID = "map"


    def __init__(self) -> None:
        for i in range(self.n1):
            for j in range(self.n2):
                self.map[i][j] = Block()
        self.obstacle_points: Set[Tuple[float, float, float]] = set() 
        rospy.init_node('publisher_and_listener', anonymous=True)
        self.pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.update_pos)
        self.rviz_rov_marker_pub = rospy.Publisher('rover_position', Marker, queue_size=10)
        self.rviz_landmark_marker_pub = rospy.Publisher('landmarks', Marker, queue_size=10)
        self.new_path = rospy.Publisher('/path', Float32MultiArray, queue_size=10)
        self.rviz_map_pub = rospy.Publisher('/gradient_grid', OccupancyGrid, queue_size=10)
        self.map_grid = {}

    def listener(self) -> None:
        rospy.Subscriber("camera/depth/points", PointCloud2, self.pcloud2_analysis)
        rospy.spin()

    def update_pos(self, msg):
        self.pos = msg.pose[1].position
        self.ori = msg.pose[1].orientation

        # If at landmark, remove it and run path update
        if abs(self.pos.x - self.landmarks[-1][0]) < self.tol_d and abs(self.pos.y - self.landmarks[-1][1]) < self.tol_d:
            self.landmarks.pop()
            self.update_path()

    def mark_pos(self):
        marker = Marker()
        marker.header.frame_id = self.FIXED_FRAME_ID
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.pos.x
        marker.pose.position.y = self.pos.y
        marker.pose.position.z = self.pos.z
        self.rviz_rov_marker_pub.publish(marker)



    def mark_landmarks(self):
        marker = Marker()
        marker.header.frame_id = self.FIXED_FRAME_ID
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        for landmark in self.landmarks:
            marker.points.append(Point(landmark[0],landmark[1],0.1))

        while (self.rviz_landmark_marker_pub.get_num_connections() < 1):
            pass
        self.rviz_landmark_marker_pub.publish(marker)
    
    def apply_camera_pose_transform(self, points_tuple: tuple, rover_position_tuple: tuple, camera_orientation_tuple: tuple) -> np.array:
        Drw = np.reshape(np.array(rover_position_tuple), (3, 1))
        Rrw = self.quaternion_rotation_matrix(camera_orientation_tuple)
        Trw = np.vstack((
            np.hstack((Rrw, Drw)), 
            np.hstack((np.zeros((1, 3)), np.array([[1]])))
        ))
        Rcr = np.array(
            [[0 ,0 ,1],
            [0 ,1 ,0],
            [-1 ,0 ,0]]
        ) @ np.array(
            [[0 ,1 ,0],
            [-1,0 ,0],
            [0 ,0 ,1]]
        )
        Dcr = np.reshape(CAMERA_POSITION_OFFSET, (3, 1))
        Tcr = np.vstack((
            np.hstack((Rcr, Dcr)),
            np.hstack((np.zeros((1, 3)), np.array([[1]])))
        ))
        P = np.vstack((
            points_tuple.T,
            np.ones((points_tuple.shape[0],))
        ))
        return Trw @ Tcr @ P

    def pcloud2_analysis(self, msg: PointCloud2) -> None:
        # skip messages which older then 1.3 sec
        if (msg.header.stamp.secs + 1.3 < rospy.get_time()):
            return

        rover_position_tuple = (self.pos.x, self.pos.y, self.pos.z)
        rover_orientation_tuple = (self.ori.x, self.ori.y, self.ori.z, self.ori.w)
        points_np = np.array(list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)))
        points_transformed_np = np.round(self.apply_camera_pose_transform(points_np, rover_position_tuple, rover_orientation_tuple).T, decimals=ROUNDING_COEF)
        
        new_obstacle_points = points_transformed_np

        # Iterate through the array and map the 2s
        for i in range(len(new_obstacle_points)):   # Iterate over rows
            current_x = new_obstacle_points[i][0]
            current_y = new_obstacle_points[i][1]
            current_z = new_obstacle_points[i][2]
            [X, Y]= self.xy_to_XY(current_x, current_y)

            current_block = self.map[X][Y]

            if current_block.avg == None:
                current_block.n = 1
                current_block.avg = current_z
            else:
                current_block.avg = (current_block.avg*current_block.n + current_z)/(current_block.n+1)
                current_block.n = current_block.n + 1

            self.update_gradients(X, Y)

        # We have updated sensor values, its time to load them to the rviz
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.FIXED_FRAME_ID
                
        MMD = MapMetaData()
        MMD.map_load_time = rospy.Time.now()
        MMD.resolution = 0.1
        MMD.width = 300
        MMD.height = 300
        MMD.origin = Pose(Point(-15,-15,0), Quaternion(0,0,0,1))

        cells = np.zeros(self.n1*self.n2, dtype=int)
        # Form Proper shaped array from map
        m = 0
        for j in range(0, self.n2):
            for i in range(0, self.n1):
                g = self.map[i][j].gradient
                if (self.map[i][j].on_path):
                    cells[m] = 127
                elif (not g == None):
                    cells[m] = int(g)
                else: 
                    cells[m] = -1
                m = m + 1
            
        self.rviz_map_pub.publish(OccupancyGrid(header, MMD, cells))
        self.mark_pos()
         
         
    # The average at X, Y was changed. This means that the gradient of 9 blocks need to be updated.
    # Also, make sure that None cases are checked. 
    def update_gradients(self, X, Y):
        max_g = 0
        for n in range(0, 8):
            dx = self.surr_8[n][0]
            dy = self.surr_8[n][1]
            # If there is an avg value
            if (not self.map[X+dx][Y+dy].avg == None):
                if (n%2 == 0):
                    g = 707*abs(self.map[X][Y].avg - self.map[X+dx][Y+dy].avg)
                else:
                    g = 1000*abs(self.map[X][Y].avg - self.map[X+dx][Y+dy].avg)
                if g > max_g:
                    max_g = g
            

            dangerous_obstacles = []
            # if there is a gradient value and there is an avg value
            if ((not self.map[X+dx][Y+dy].gradient == None) and (not self.map[X+dx][Y+dy].avg == None)):
                if (g > self.map[X+dx][Y+dy].gradient):
                    if (g < 100):
                        self.map[X+dx][Y+dy].gradient = g
                    else:
                        if self.map[X+dx][Y+dy].on_path:
                            dangerous_obstacles.append([X+dx, Y+dy])
                        self.map[X+dx][Y+dy].gradient = 100
                        for j in range(0,56):
                            dxx = self.surr_40[j][0]
                            dyy = self.surr_40[j][1]
                            self.map[X+dx + dxx][Y+dy+dyy].gradient = 100
                            if self.map[X+dx + dxx][Y+dy+dyy].on_path:
                                dangerous_obstacles.append([X+dx + dxx, Y+dy+dyy])

        old_g = self.map[X][Y].gradient

        if (max_g < 100):
            self.map[X][Y].gradient = max_g
        else:
            if self.map[X][Y].on_path:
                dangerous_obstacles.append([X, Y])
            self.map[X][Y].gradient = 100
            for i in range(0, 56):
                dxxx = self.surr_40[i][0]
                dyyy = self.surr_40[i][1]
                self.map[X+dxxx][Y+dyyy].gradient = 100
                if self.map[X+dxxx][Y+dyyy].on_path:
                    dangerous_obstacles.append([X+dxxx, Y+dyyy])

        if (not old_g == None):
            if (old_g > max_g):
                self.map[X][Y].gradient = old_g

        if not len(dangerous_obstacles) == 0:
            self.update_path()
    
    # Find the path, send it to moving.py
    def update_path(self):
        #   While the path is  being updated, the rover must be stopped
        s = Float32MultiArray()
        s.data = []

        while (self.new_path.get_num_connections() < 1):
            pass
        self.new_path.publish(s)

        x1 = self.pos.x
        y1 = self.pos.y

        x2 = self.landmarks[-1][0]
        y2 = self.landmarks[-1][1]

        A = self.xy_to_XY(x1, y1)
        B = self.xy_to_XY(x2, y2)

        new_path_XY = self.pathfinding(tuple(A), tuple(B))

        # new_path_XY contains the entire path, update the on_path fields.
        # Nuke old fields
        if (not self.cur_p == None):
            for point in self.cur_p:
                X = point[0]
                Y = point[1]
                self.map[X][Y].on_path = False

        self.cur_p = new_path_XY

        # Enforce new fields
        for point in self.cur_p:
            X = point[0]
            Y = point[1]
            self.map[X][Y].on_path = True

        new_path_XY = self.path_reduction(new_path_XY)

        sent_array = []
        for point in new_path_XY:
            [x, y] = self.XY_to_xym(point[0], point[1])
            sent_array.append(x)
            sent_array.append(y)
        f = Float32MultiArray()
        f.data = sent_array

        while (self.new_path.get_num_connections() < 1):
            pass
        self.new_path.publish(f)
            


    def quaternion_rotation_matrix(self, Q: tuple):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[3]
        q1 = Q[0]
        q2 = Q[1]
        q3 = Q[2]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
    

    def xy_to_XY(self, x, y):
        return [int(10*math.floor(x) + self.n1/2) +  math.floor(x%1 *10), int(10*math.floor(y) + self.n2/2) +  math.floor(y%1 *10)]

    def XY_to_xym(self, X, Y):
        return [(0.1*X) - 15 + 0.05, (0.1*Y) - 15 + 0.05]

    # A is the grid coordinates of starting position
    # B is the grid coordinates of ending position
    # This method uses a modified A* pathing algorithm allowing diagonal movement
    # The inputs are Tuples
    def pathfinding(self, A, B):
        self.openset = PriorityQueue()
        self.node_table.clear()
        self.openset.put((0, A))
        self.node_table[A] = [0, None]

        while True:
            cur_node = self.openset.get()[1]
            if (cur_node == B):
                return self.pathing_done(A, B)
            self.update_neighbors(cur_node, B)

    # Updates openset and node_table from neighboors of Node. 
    # It makes sure that the node is not out of bounds, and it
    # is not an obstacle
    def update_neighbors(self, Node, B):
        X = Node[0]
        Y = Node[1]
        G = self.node_table[Node][0]
        for n in range(0, 8):
            dx = self.surr_8[n][0]
            dy = self.surr_8[n][1]
            
            if ((X + dx >= 0) and (X + dx <= self.n1) and (Y + dy >= 0) and (Y + dy <= self.n2) and ((self.map[X+dx, Y+dy].gradient == None) or (self.map[X+dx, Y+dy].gradient < 100))):
                cur_neigh = (X+dx, Y+dy)

                # Calculate H
                H = self.h_score(cur_neigh, B)

                if (n%2 == 0):
                    G_try = G + 1.414
                else:
                    G_try = G + 1
                
                try:
                    if (self.node_table[cur_neigh][0] > G_try):
                        self.node_table[cur_neigh] = [G_try, Node]

                        # Update openset 
                        entries = []
                        while not self.openset.empty():
                            [priority, item] = self.openset.get()
                            if item == cur_neigh:
                                break
                            entries.append((priority, item))

                        # Then, add it again with the new priority
                        self.openset.put(((H + G_try), cur_neigh))

                        for priority, item in entries:
                            self.openset.put((priority, item))

                except KeyError:
                    self.node_table[cur_neigh] = [G_try, Node]
                    self.openset.put((H + G_try, cur_neigh))


    def pathing_done(self, A, B):
        path_array = [B]
        cur = B
        path_array.append(B)
        while True:
            prev = self.node_table[cur][1]
            if (prev == A):
                break
            path_array.append(prev)
            cur = prev
        # path_array.reverse()
        return path_array

    # H_score of a node is its distance from destination
    def h_score(self, Node, B):
        dx = Node[0] - B[0]
        dy = Node[1] - B[1]
        return math.sqrt(dx*dx + dy*dy)
    
    # Reduces the path to a minimum of instructions
    def path_reduction(self, full_path):
        n1 = full_path[0]
        n2 = full_path[1]
        dx1 = n2[0] - n1[0]
        dy1 = n2[1] - n1[1]

        red_path = []
        for n in range(2, len(full_path)):
            n1 = n2
            n2 = full_path[n]

            dx2 = n2[0] - n1[0]
            dy2 = n2[1] - n1[1]

            if (not (dx1 == dx2 and dy1 == dy2)): # if direction change
                red_path.append(n1)

            dx1 = dx2
            dy1 = dy2
        
        red_path.append(full_path[len(full_path) - 1])
        return red_path

if __name__ == '__main__':
    trcker = PointCloudTracker()
    trcker.mark_landmarks()
    trcker.update_path()
    trcker.listener()


