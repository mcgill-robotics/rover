#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from autonomy_config import CAMERA_POSITION_OFFSET, ROUNDING_COEF
import numpy as np
from typing import Set, Tuple, List
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from queue import PriorityQueue
import message_filters
from time import sleep

class Block:
    def __init__(self):
        self.avg = None  
        self.n = 0  
        self.gradient = None
        self.buffer = -1
        self.on_path = False
        self.last_update = 0

class PointCloudTracker:
    gps_n = 0
    initial_gps = [0, 0] # Value is only good when gps_n = 100

    n1 = 600
    n2 = 600
    r = 0.1
    map = np.empty((n1, n2), dtype=Block)

    surr_8 = [[-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1]]
    surr_40 = [[0, 5], [1, 5], [2, 5], [3, 5], [3, 4], [4, 4], [4, 3], [5, 3], [5, 2], 
              [5, 1], [5, 0], [5, -1], [5, -2], [5, -3], [4, -3], [4, -4], [3, -4], 
              [3, -5], [2, -5], [1, -5], [0, -5],[-1, 5], [-2, 5], [-3, 5], [-3, 4], 
              [-4, 4], [-4, 3], [-5, 3], [-5, 2], [-5, 1], [-5, 0], [-5, -1], [-5, -2], 
              [-5, -3], [-4, -3], [-4, -4], [-3, -4], [-3, -5], [-2, -5], [-1, -5]]
    # surr_40 = [[0,7],[1,7],[2,7],[3,7],[4,7],[4,6],[5,6],[6,6],[6,5],[6,4],[7,4],[7,3],
    #            [7,2],[7,1],[7,0],[7,-1],[7,-2],[7,-3],[7,-4],[6,-4],[6,-5],[6,-6],[5,-6],
    #            [4,-6],[4,-7],[3,-7],[2,-7],[1,-7],[0,-7],[-1,7],[-2,7],[-3,7],[-4,7],[-4,6],
    #            [-5,6],[-6,6],[-6,5],[-6,4],[-7,4],[-7,3],[-7,2],[-7,1],[-7,0],[-7,-1],[-7,-2],
    #            [-7,-3],[-7,-4],[-6,-4],[-6,-5],[-6,-6],[-5,-6],[-4,-6],[-4,-7],[-3,-7],[-2,-7],[-1,-7]]
    surr_12 = [[-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [2, 0], [-2, 0], [0, 2], [0, -2]]

    bufferF_or_gradientT_display = True # F = buffer, T = Gradient

    # Current Position
    pos = Point()

    # Current Orientation
    ori = Quaternion()

    # Landmark arrival tolerance
    tol_d = 1.2

    # Contains open nodes, and their -F score to keep order
    openset = PriorityQueue()

    # Node table. Contains all known nodes as "key" and their G score, and previous node as "values"
    node_table = dict()

    # Entire Landmarks Left
    landmarks = [[45.536465, -73.630130]] # 11,6
    # [45.536503, -73.630101]
    
    # Current full path
    cur_p = None

    # Particular POINT
    pp_x = None
    pp_y = None

    # Distance from pp to nearest obstacle
    dist_pp_nobs = None

    # Frame ID
    FIXED_FRAME_ID = "map"

    # Current Point Cloud ID
    pcid = 0

    # List of obstacles found on path
    dangerous_obstacles = []

    # Buffer Levels
    BUFFER_LVL_2 = 100
    BUFFER_LVL_1 = 65
    BUFFER_LVL_0 = 0

    # Obstacle Sensitivity
    OBS_SENS = 0.79 # 0.45

    def __init__(self) -> None:
        for i in range(self.n1):
            for j in range(self.n2):
                self.map[i][j] = Block()
        self.obstacle_points: Set[Tuple[float, float, float]] = set() 
        rospy.init_node('publisher_and_listener', anonymous=True)
        self.gps_sub = rospy.Subscriber("fake/gps", Float32MultiArray, self.gps_handler)
        
    def pubs_and_list(self):
        self.pose_sub = rospy.Subscriber("/gazebo/zed2/odom", Odometry, self.update_pos)
        self.rviz_rov_marker_pub = rospy.Publisher('rover_position', Marker, queue_size=10)
        self.rviz_landmark_marker_pub = rospy.Publisher('landmarks', Marker, queue_size=10)
        self.new_path = rospy.Publisher('/path', Float32MultiArray, queue_size=10)
        self.rviz_map_pub = rospy.Publisher('/gradient_grid', OccupancyGrid, queue_size=10)
        self.map_grid = {}

    def cam_listener(self) -> None:
        data_pc = message_filters.Subscriber("zed2/point_cloud/cloud_registered", PointCloud2)
        pose_pc = message_filters.Subscriber("/gazebo/zed2/odom", Odometry)
        ts = message_filters.TimeSynchronizer([data_pc, pose_pc], 5)
        ts.registerCallback(self.pcloud2_analysis)
        rospy.spin()

    def gps_handler(self, msg):
        if self.gps_n == 100:
            return
        self.gps_n = self.gps_n + 1
        self.initial_gps[0] = self.initial_gps[0] + msg.data[0]
        self.initial_gps[1] = self.initial_gps[1] + msg.data[1]
        if self.gps_n == 100:
            self.initial_gps[0] = self.initial_gps[0]/100
            self.initial_gps[1] = self.initial_gps[1]/100

    def convert_landmarks(self):
        while self.gps_n < 100:
            pass
        
        ref_lat_rad = math.radians(self.initial_gps[0])
        ref_lon_rad = math.radians(self.initial_gps[1])
        for landmark in self.landmarks:
            lat_rad = math.radians(landmark[0])
            lon_rad = math.radians(landmark[1])

            # Calculate x and y
            landmark[0] = 6371000 * (lat_rad - ref_lat_rad)
            landmark[1] = 6371000 * (lon_rad - ref_lon_rad) * math.cos(ref_lat_rad)
    
    def update_pos(self, msg):
        self.pos = msg.pose.pose.position
        self.ori = msg.pose.pose.orientation

        # Update Path if you got close enough
        if (not self.dist_pp_nobs == None):
            if (self.dist_pp_nobs < 0.8):
                self.dist_pp_nobs = None
                self.pp_x = None
                self.pp_y = None
                self.update_path()
            else:
                dx = self.pos.x - self.pp_x
                dy = self.pos.y - self.pp_y
                curr_dist = math.sqrt(dx*dx + dy*dy)
                if (curr_dist > self.dist_pp_nobs/3):
                    self.dist_pp_nobs = None
                    self.pp_x = None
                    self.pp_y = None
                    self.update_path()

        if (len(self.landmarks) == 0):
            exit()
        # If at landmark, remove it and run path update
        if abs(self.pos.x - self.landmarks[-1][0]) < self.tol_d and abs(self.pos.y - self.landmarks[-1][1]) < self.tol_d:
            self.landmarks.pop()
            if (not len(self.landmarks) == 0):
                self.update_path()
            else: 
                exit()

    def mark_pos(self):
        marker = Marker()
        marker.header.frame_id = self.FIXED_FRAME_ID
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = self.ori.w
        marker.pose.orientation.x = self.ori.x
        marker.pose.orientation.y = self.ori.y
        marker.pose.orientation.z = self.ori.z

        marker.pose.position.x = self.pos.x
        marker.pose.position.y = self.pos.y
        marker.pose.position.z = 0.1
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
        marker.pose.orientation.w = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        for landmark in self.landmarks:
            marker.points.append(Point(landmark[0],landmark[1],0.1))

        while (self.rviz_landmark_marker_pub.get_num_connections() < 1):
            pass
        self.rviz_landmark_marker_pub.publish(marker)
    
    def apply_camera_pose_transform(self, points_tuple: tuple, rover_position_tuple: tuple, camera_orientation_tuple: tuple) -> np.array:
        Drw = np.reshape(np.array(rover_position_tuple), (3, 1))
        Rrw = self.quaternion_rotation_matrix(self.quaternion_multiplication(camera_orientation_tuple, (0, 0.2064604, 0, 0.978455)))
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
    
    def quaternion_multiplication(self, quaternion1, quaternion0):
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return (x1*w0 + y1*z0 - z1*y0 + w1*x0,
                -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                x1*y0 - y1*x0 + z1*w0 + w1*z0,
                -x1*x0 - y1*y0 - z1*z0 + w1*w0)

    def pcloud2_analysis(self, msg1, msg2) -> None:
        # skip messages which older then 1.1 sec
        if (msg1.header.stamp.secs + 1 < rospy.get_time()):
            return

        # make sure angular speed was small enough else there will be much of an error on the pointclouds position
        # if (abs(msg2.twist.twist.angular.x) + abs(msg2.twist.twist.angular.y) + abs(msg2.twist.twist.angular.z) > 0.2):
        #    return
        self.pcid = self.pcid + 1
        p = msg2.pose.pose.position
        o = msg2.pose.pose.orientation
        rover_position_tuple = (p.x, p.y, p.z)
        rover_orientation_tuple = (o.x, o.y, o.z, o.w)

        points_np = np.array(list(pc2.read_points(msg1, field_names=['x', 'y', 'z'], skip_nans=True)))
        points_transformed_np = np.round(self.apply_camera_pose_transform(points_np, rover_position_tuple, rover_orientation_tuple).T, decimals=ROUNDING_COEF)
        
        new_obstacle_points = points_transformed_np
        # Iterate through the array and map the 2s
        changed_points = set()
        for i in range(len(new_obstacle_points)):   # Iterate over rows
            if (not i%20 == 0): # Only use 1 point out if 10
                continue
            current_x = new_obstacle_points[i][0]
            current_y = new_obstacle_points[i][1]
            current_z = new_obstacle_points[i][2]
            [X, Y]= self.xy_to_XY(current_x, current_y)

            if (X < (self.n1-8) and Y < (self.n2-8) and X > 8 and Y > 8):
                current_block = self.map[X][Y]

                if current_block.avg == None:
                    current_block.n = 1
                    current_block.avg = current_z
                    current_block.last_update = self.pcid
                else:
                    if not current_block.last_update == self.pcid: 
                        current_block.n = int(1.05*current_block.n) # Reduce weight of older points
                        current_block.last_update = self.pcid
                    current_block.avg = (current_block.avg*current_block.n + current_z)/(current_block.n+1)
                    current_block.n = current_block.n + 1
                    if (current_block.n%10 == 0): # Only send the point to be updated if n is a multiple of 10
                        changed_points.add((X,Y))
                    

        for point in changed_points:
            (X, Y) = point
            self.update_gradients(X, Y, p)
            # Find which obstacle was the closest
            if not len(self.dangerous_obstacles) == 0:
                if (self.pp_x == None):
                    self.pp_x = p.x
                    self.pp_y = p.y
                for point in self.dangerous_obstacles:
                    [xo , xy] = self.XY_to_xym(point[0], point[1])
                    dx = xo - self.pp_x
                    dy = xy - self.pp_y
                    if (self.dist_pp_nobs == None):
                        self.dist_pp_nobs = math.sqrt(dx*dx + dy*dy)
                    elif (self.dist_pp_nobs > math.sqrt(dx*dx + dy*dy)):
                        self.dist_pp_nobs = math.sqrt(dx*dx + dy*dy)

        # We have updated sensor values, its time to load them to the rviz
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.FIXED_FRAME_ID
                
        MMD = MapMetaData()
        MMD.map_load_time = rospy.Time.now()
        MMD.resolution = self.r
        MMD.width = self.n1
        MMD.height = self.n2
        MMD.origin = Pose(Point(-self.n1*0.5*self.r,-self.n2*0.5*self.r,0), Quaternion(0,0,0,1))

        cells = np.zeros(self.n1*self.n2, dtype=int)
        # Form Proper shaped array from map
        m = 0
        for j in range(0, self.n2):
            for i in range(0, self.n1):
                if self.bufferF_or_gradientT_display:
                    g = self.map[i][j].gradient
                    if (self.map[i][j].on_path):
                        cells[m] = 127
                    elif (not g == None):
                        if g > 100:
                            g = 100
                        cells[m] = int(g)
                    else: 
                        cells[m] = -1
                else: 
                    g = self.map[i][j].buffer
                    if (self.map[i][j].on_path):
                        cells[m] = 127
                    elif (g == self.BUFFER_LVL_2):
                        cells[m] = -128
                    elif (g == self.BUFFER_LVL_1): 
                        cells[m] = -50
                    else:
                        cells[m] = g
                m = m + 1
        self.rviz_map_pub.publish(OccupancyGrid(header, MMD, cells))
        self.mark_pos()
         
         
    # The average at X, Y was changed. This means that the gradient of 9 blocks need to be updated.
    # Also, make sure that None cases are checked. Assume that if the is less than 5 samples to form the average
    # It is not enough to be significant. We know for a fact that at [X][Y] there is 5 because its only called if
    # there is at least 5. You only need to change others.
    def update_gradients(self, X, Y, p):
        max_g = 0
        surr_g = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.dangerous_obstacles = []
        for n in range(0, 8):
            dx = self.surr_8[n][0]
            dy = self.surr_8[n][1]
            # If there is a avg value
            if (not self.map[X+dx][Y+dy].avg == None and self.map[X+dx][Y+dy].n >= 10):
                if (n%2 == 0):
                    g = self.OBS_SENS*707*(self.map[X][Y].avg - self.map[X+dx][Y+dy].avg) #0.5
                else:
                    g = self.OBS_SENS*1000*(self.map[X][Y].avg - self.map[X+dx][Y+dy].avg) #0.5
                surr_g[n] = g
                g = abs(g)

            # if there is a gradient value and there is an avg value
            if ((not self.map[X+dx][Y+dy].gradient == None) and (not self.map[X+dx][Y+dy].avg == None) and (self.map[X+dx][Y+dy].n >= 10)):
                if (g > self.map[X+dx][Y+dy].gradient): # Gradient needs to be changed
                    if (g < self.BUFFER_LVL_1):
                        self.map[X+dx][Y+dy].gradient = g
                        if self.BUFFER_LVL_0 > self.map[X+dx][Y+dy].buffer:
                            self.map[X+dx][Y+dy].buffer = self.BUFFER_LVL_0
                    elif (g < self.BUFFER_LVL_2):
                        self.map[X+dx][Y+dy].gradient = g
                        self.buffer_create(X+dx, Y+dy, self.BUFFER_LVL_1, p)
                    else:
                        self.map[X+dx][Y+dy].gradient = g
                        self.buffer_create(X+dx, Y+dy, self.BUFFER_LVL_2, p)

        # Compute max_g
        max_g = abs(max(surr_g, key=abs))
        for n in range(0, 8):
            g0 = surr_g[n]    
            g3 = surr_g[(n + 3)%8] 
            g4 = surr_g[(n + 4)%8]
            g5 = surr_g[(n + 5)%8]
            g_op = max(0.65*abs(g0 + g3), 0.65*abs(g0 + g4), 0.65*abs(g0 + g5))
            if (g_op > max_g):
                max_g = g_op

        if (max_g < self.BUFFER_LVL_1):
            self.map[X][Y].gradient = max_g
            if self.BUFFER_LVL_0 > self.map[X][Y].buffer:
                self.map[X][Y].buffer = self.BUFFER_LVL_0
        elif (max_g < self.BUFFER_LVL_2):
            self.map[X][Y].gradient = max_g
            self.buffer_create(X, Y, self.BUFFER_LVL_1, p)
        else:
            self.map[X][Y].gradient = max_g
            self.buffer_create(X, Y, self.BUFFER_LVL_2, p)

    # Helper Function to create buffer
    def buffer_create(self, X, Y, buffer_lvl, p):
        for j in range(0,40):
            dxx = self.surr_40[j][0]
            dyy = self.surr_40[j][1]
            if buffer_lvl > self.map[X+dxx][Y+dyy].buffer:
                self.map[X+dxx][Y+dyy].buffer = buffer_lvl
            if buffer_lvl >= self.BUFFER_LVL_1 and self.map[X+dxx][Y+dyy].on_path:
                self.dangerous_obstacles.append((X+dxx, Y+dyy))
                if (((p.x - X+dxx)**2 + (p.y - Y+dyy)**2) < 0.64):
                    s = Float32MultiArray()
                    s.data = []
                    while (self.new_path.get_num_connections() < 1):
                        pass
                    self.new_path.publish(s)
            
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
        new_path_XY = self.pathfinding(tuple(A), tuple(B), self.BUFFER_LVL_1)
        if (not new_path_XY[0]):
            print('Allowing Yellow')
            new_path_XY = self.pathfinding(tuple(A), tuple(B), self.BUFFER_LVL_2)
        if (new_path_XY[0]):
            new_path_XY = new_path_XY[1]
        else: 
            print("No Path Available")
            exit()
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
        return [int(math.floor(x)/self.r + self.n1/2) +  math.floor((x%1) /self.r), int(math.floor(y)/self.r + self.n2/2) +  math.floor((y%1) /self.r)]

    def XY_to_xym(self, X, Y):
        return [(self.r*X) - self.n1*0.5*self.r + self.r/2, (self.r*Y) - self.n2*0.5*self.r + self.r/2]

    # A is the grid coordinates of starting position
    # B is the grid coordinates of ending position
    # This method uses a modified A* pathing algorithm allowing diagonal movement
    # The inputs are Tuples
    # Returns (True, path) if it worked or (False, None) if it didnt
    def pathfinding(self, A, B, unallowed):
        self.openset = PriorityQueue()
        self.node_table.clear()
        self.openset.put((0, A))
        self.node_table[A] = [0, None]

        while not self.openset.empty():
            cur_node = self.openset.get()[1]
            if (abs(cur_node[0] - B[0]) <= (1/self.r) and abs(cur_node[1] - B[1]) <= (1/self.r)): # If you are close enough to landmark pathing is done
                return (True, self.pathing_done(A, cur_node))
            self.update_neighbors(cur_node, B, unallowed)
        return (False, None)

    # Updates openset and node_table from neighboors of Node. 
    # It makes sure that the node is not out of bounds, and it
    # is not an obstacle
    def update_neighbors(self, Node, B, unallowed):
        X = Node[0]
        Y = Node[1]
        G = self.node_table[Node][0]
        for n in range(0, 8):
            dx = self.surr_8[n][0]
            dy = self.surr_8[n][1]
            
            if ((X + dx >= 0) and (X + dx < self.n1) and (Y + dy >= 0) and (Y + dy < self.n2) and (self.map[X+dx, Y+dy].buffer < unallowed)):
                cur_neigh = (X+dx, Y+dy)

                # Calculate H
                H = self.h_score(cur_neigh, B)

                if (n%2 == 0):
                    G_try = G + 1.4142
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
        n2 = full_path[2]
        dx1 = n2[0] - n1[0]
        dy1 = n2[1] - n1[1]
        red_path = []
        for n in range(2, int(len(full_path)/2)):
            n1 = n2
            n2 = full_path[2*n]
            dx2 = n2[0] - n1[0]
            dy2 = n2[1] - n1[1]
            red_path.append(n1)
            if (not (dx1 == dx2 and dy1 == dy2)): # if direction change
                red_path.append(full_path[2*n-1])
            red_path.append(n2)
            dx1 = dx2
            dy1 = dy2
        red_path.append(full_path[len(full_path) - 1])
        return red_path

if __name__ == '__main__':
    tracker = PointCloudTracker()
    tracker.convert_landmarks()
    # print(tracker.landmarks)
    tracker.pubs_and_list()
    tracker.mark_landmarks()
    tracker.update_path()
    tracker.cam_listener()

