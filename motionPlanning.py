# This was developed using python2.7+
# Please read the ReadMe.md for set up environment

import time
import math
import random
import numpy as np
import b0RemoteApi

random.seed(2)
np.seterr(divide='ignore', invalid='ignore')

# set positions

start_pos = [0,0,0,0,0,0]

positions = []
positions.append([-83.2,    -59.6,  87.9,   413.1,  -252.8, -246.6])
positions.append([6.55,     -14.7,  -74,    618.1,  -60.1,  -573.3])

positions.append([-90.9,    -1.7,   76.6,   126.2,  -110.8, -42.19])
positions.append([-59.4,    53.4,   -55,    204.6,  -445,   -348.1])

positions.append([6.55,     -14.7,  -74,    618.1,  -60.1,  -573.3])
positions.append([-111.8,   43.8,   -92,    173.5,  -390,   -142.4])

positions.append([-59.4,    53.4,   -55,    204.6,  -445,   -348.1])
positions.append([-83.2,    -59.6,  87.9,   413.1,  -252.8, -246.6])

positions.append([-111.8,   43.8,   -92,    173.5,  -390,   -142.4])
positions.append([-90.9,    -1.7,   76.6,   126.2,  -110.8, -42.19])


# convert positions to radians
for i in range(0, len(positions)):
    for j in range(0, len(positions[i])):
        positions[i][j] = positions[i][j] * math.pi / 180

# helper for converting DH parameters to matrix
def dEnAviTHarTeNbErg(lst):
    linkTwist  = lst[0] #alpha
    linkLength = lst[1] #a
    linkOffset = lst[2] #d
    jointAngle = lst[3] #theta

    M = np.matrix([
        [math.cos((jointAngle)),                         -math.sin((jointAngle)),                        0,                      linkLength],
        [math.sin((jointAngle)) * math.cos((linkTwist)), math.cos((jointAngle)) * math.cos((linkTwist)), -math.sin((linkTwist)), -(linkOffset * math.sin((linkTwist)))],
        [math.sin((jointAngle)) * math.sin((linkTwist)), math.cos((jointAngle)) * math.sin((linkTwist)), math.cos((linkTwist)),  linkOffset * math.cos((linkTwist))],
        [0,                                              0,                                              0,                      1]])

    return(M)

def forwardKinematicsDH(angles):
    
    # define parameters
    allValues = []
    allValues.append([180*math.pi/180,  0,           -275.5/1000.0,     angles[0] + 90*math.pi/180])
    allValues.append([90*math.pi/180,   0,           0,                 angles[1] + 90*math.pi/180])
    allValues.append([180*math.pi/180,  -290/1000.0, -7/1000.0,         angles[2] + 90*math.pi/180])
    allValues.append([90*math.pi/180,   0,           -166.03/1000.0,    angles[3]])
    allValues.append([60*math.pi/180,   0,           -85.56/1000.0,     angles[4]])
    allValues.append([-60*math.pi/180,  0,           0,                 angles[5]])
    allValues.append([180*math.pi/180,  0,           202.78/1000.0,     90*math.pi/180])

    # calculate resultant matrix
    pose = dEnAviTHarTeNbErg(allValues[0])
    for i in range(6):
        pose *= dEnAviTHarTeNbErg(allValues[i + 1])

    return pose

# set the position of the robot in V-REP accounting for angle differences
def setAllJointPositions(angles):
    client.simxSetJointPosition(joint1Handle,angles[0] - math.pi/2,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint2Handle,angles[1] + math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint3Handle,angles[2] + math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint4Handle,angles[3],client.simxDefaultPublisher())
    client.simxSetJointPosition(joint5Handle,angles[4],client.simxDefaultPublisher())
    client.simxSetJointPosition(joint6Handle,angles[5],client.simxDefaultPublisher())

# check if angle configuration collides with any obstacles in V-REP by moving the arm by those angles
def isCollisionFree(angles):
    setAllJointPositions(angles)
    _, obstaclesHandle = client.simxGetCollectionHandle("Obstacles#0", client.simxServiceCall())
    _, isColliding = client.simxCheckCollision(obstaclesHandle, "sim.handle_all", client.simxServiceCall())
    return not isColliding

# return a random valid robot comfiguration
def sampleRobotConfiguration():
    angles = []
    angles.append(random.uniform(-2 * math.pi, 2 * math.pi))
    angles.append(random.uniform(-2.32, 2.31))
    angles.append(random.uniform(-2.81, 2.80))
    angles.append(random.uniform(-2 * math.pi, 2 * math.pi))
    angles.append(random.uniform(-2 * math.pi, 2 * math.pi))
    angles.append(random.uniform(-2 * math.pi, 2 * math.pi))
    return angles

# returns euclidean distance between two points
def dist(a, b=[]):
    if not b:
        b = [0 for _ in range(len(a))]
    return (sum([(a[i] - b[i])**2 for i in range(len(a))]))**0.5

# draw sphere at the end of the end affector using DH parameters
def drawSphereDH(angles):
    eePoseFK_DH = forwardKinematicsDH(angles) # calculate end pose
    eePositionFK = [eePoseFK_DH[0,3], eePoseFK_DH[1,3], eePoseFK_DH[2,3]] # get end effector position
    _, visEEHandle = client.simxAddDrawingObject_spheres(0.01, [0,125,0], eePositionFK, client.simxServiceCall())
    return visEEHandle # after drawing, return object for future deletion
        
# draw all of the planned motions
def runConfigurationSequence(plannedMotions):
    # setAllJointPositions(start_pos)
    for i in range(len(plannedMotions)):
        time.sleep(0.1)
        setAllJointPositions(plannedMotions[i])

class Node(object):

    def __init__(self, coords, parent):
        assert len(coords) == 6
        self.coords = coords
        self.parent = parent
        self.children = []
    
    def __eq__(self, node):
        return node and self.coords == node.coords
    
    def __hash__(self):
        return hash(tuple(self.coords))

    def __str__(self):
        return "Node:" + str(self.coords)

    def __repr__(self):
        return "<Node coords:%s>" % (self.coords)

    def euclidean_distance(self, node):
        assert (node)
        return (sum([(self.coords[i] - node.coords[i])**2 for i in range(len(self.coords))]))**0.5

class RRT(object):

    def __init__(self, space, dimensionality):
        assert space.shape == (dimensionality, 2)
        self.space = space
        self.dim = dimensionality

    def node_is_free(self, node):
        return isCollisionFree(node.coords)

    def sample_node(self):
        return Node(sampleRobotConfiguration(), None)

    # return list of nodes from start to end by interating through last node's parent pointers
    def _follow_parent_pointers(self, node):
        curr_ptr = node
        path = [node]
        while curr_ptr is not None:
            path.append(curr_ptr)
            curr_ptr = curr_ptr.parent
        return path[::-1]

    def find_closest_node(self, tree, curr_node):
        min_dist = float("Inf")
        closest_node = None
        for node in tree:
            dist = node.euclidean_distance(curr_node)  
            if dist < min_dist:
                closest_node = node
                min_dist = dist
        return closest_node

    # return a new node on the line from n_nearest to n_rand, and the new node is at most step_size away from n_nearest
    def step_towards(self, n_nearest, n_rand, step_size):
        coords = n_rand.coords
        dist = n_nearest.euclidean_distance(n_rand)
        if (dist > step_size):
            mult = step_size / dist
            coords = [np.clip(n_nearest.coords[i] + (n_rand.coords[i] - n_nearest.coords[i]) * mult, self.space[i][0], self.space[i][1]) for i in range(self.dim)]
        n_new = Node(coords, n_nearest)
        return n_new
    
    # linear method
    def path_is_obstacle_free(self, n_from, n_to, granularity=1):
        if not self.node_is_free(n_to):
            return False
        dist = [n_to.coords[i] - n_from.coords[i] for i in range(6)]
        maximum = abs(max(dist, key=abs))
        num_steps = int(maximum / granularity)
        step = [dist[i] / num_steps for i in range(6)]
        n_curr = Node(n_from.coords, None)
        for _ in range(num_steps):
            n_curr.coords = [n_curr.coords[i] + step[i] for i in range(self.dim)]
            if not self.node_is_free(n_curr):
                return False
            
        return True

    # optimized, non-sequential method
    def path_is_obstacle_free_optimized(self, n_from, n_to, granularity=1):
        if not self.node_is_free(n_to):
            return False
        dist = [n_to.coords[i] - n_from.coords[i] for i in range(self.dim)]
        sampled_nodes = [n_from]
        while sum([x**2 for x in dist]) > granularity:
            dist = [x/2 for x in dist]
            new_nodes = []
            for node in sampled_nodes:
                new_node = Node([node.coords[i] + dist[i] for i in range(self.dim)], None)
                if not self.node_is_free(new_node):
                    return False
                new_nodes.append(new_node)
            sampled_nodes += new_nodes
        return True

    
    def plan(self, start_node, dest_node, max_num_steps, step_size, dest_reached_dist):

        # The set containing the nodes of the tree
        tree = set()
        tree.add(start_node)

        plan = [start_node]
        
        for _ in xrange(max_num_steps):

            # compute s_new
            n_rand = self.sample_node()
            n_nearest = self.find_closest_node(tree, n_rand)
            n_new = self.step_towards(n_nearest, n_rand, step_size)
            
            # add n_new to tree, if a free path from n_nearest to n_new exists
            if self.path_is_obstacle_free_optimized(n_nearest, n_new, 0.1):
                tree.add(n_new)
                n_nearest.children.append(n_new)

                # if node is close to destination and path to destination is clear, return set of points from start to dest
                if n_new.euclidean_distance(dest_node) < dest_reached_dist and self.path_is_obstacle_free_optimized(n_new, dest_node):
                    dest_node.parent = n_new
                    plan = self._follow_parent_pointers(dest_node)
                    return plan

        # return if at the starting config, or if max number of steps is reached
        return [start_node]

#We're doing RRT
def planMotion(start_coords, dest_coords):
    
    space = np.array([[-2 * math.pi, 2 * math.pi],
                      [-2.32, 2.31],
                      [-2.81, 2.80],
                      [-2 * math.pi, 2 * math.pi],
                      [-2 * math.pi, 2 * math.pi],
                      [-2 * math.pi, 2 * math.pi]])

    rrt = RRT(space, 6)

    start_node = Node(start_coords, None)
    dest_node = Node(dest_coords, None)
    max_num_steps = 1000
    step_size = 30
    dest_reached_dist = 50

    plan = rrt.plan(start_node, dest_node, max_num_steps, step_size, dest_reached_dist)
    motion = [x.coords for x in plan]
    return motion

# create set of points between given set of coordinates
def linearizeMotion(coords_list, granularity=1):
    motion = []
    for current_coord in coords_list:
        if motion and current_coord != motion[-1]:
            dist = [(current_coord[i] - motion[-1][i]) for i in range(6)]
            maximum = abs(max(dist, key=abs))
            num_steps = int(maximum / granularity)
            step = [dist[i] / num_steps for i in range(6)]
            for _ in range(num_steps):
                motion.append([motion[-1][i] + step[i] for i in range(6)])
        motion.append(current_coord)
    return motion


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:    
    doNextStep=True
    
    #Create callback functions for certain events
    def simulationStepStarted(msg):
        simTime = msg[1][b'simulationTime']
        print('Simulation step started. Simulation time: ', simTime)
        
    def simulationStepDone(msg):
        simTime = msg[1][b'simulationTime']
        print('Simulation step done. Simulation time: ', simTime)
        global doNextStep
        doNextStep=True
        
    client.simxSynchronous(True)
    
    #Register callbacks
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
    
    #Get joint handles
    error,joint1Handle=client.simxGetObjectHandle('Mico_joint1',client.simxServiceCall())
    error,joint2Handle=client.simxGetObjectHandle('Mico_joint2',client.simxServiceCall())
    error,joint3Handle=client.simxGetObjectHandle('Mico_joint3',client.simxServiceCall())
    error,joint4Handle=client.simxGetObjectHandle('Mico_joint4',client.simxServiceCall())
    error,joint5Handle=client.simxGetObjectHandle('Mico_joint5',client.simxServiceCall())
    error,joint6Handle=client.simxGetObjectHandle('Mico_joint6',client.simxServiceCall())
    
    #Get end-effector handle
    error,eeHandle=client.simxGetObjectHandle('MicoHand_Dummy2',client.simxServiceCall())


    # calculate motions and plot show results

    print "Planning motions with RRT..."
    for i in range(5):
        motions1 = []
        motions2 = []
        
        motions1.append(planMotion(start_pos, positions[i * 2]))
        motions2.append(planMotion(start_pos, positions[(i * 2) +1]))
        print "Found", len(motions1[-1]), "nodes for last motion."

        lin_motions1 = [linearizeMotion(m, 0.1) for m in motions1]
        lin_motions2 = [linearizeMotion(m, 0.1) for m in motions2]

        raw_input("-- Press anything to visualize --")
        for m in lin_motions1:
            runConfigurationSequence(m[::-1])
        for m in lin_motions2:
            runConfigurationSequence(m)

        raw_input("-- Press anything to continue --")
    
    # motions1 = []
    # motions2 = []
    
    # motions1.append(planMotion(start_pos, positions[0]))
    # motions2.append(planMotion(start_pos, positions[1]))
    # print "Found", len(motions1[-1]), "nodes for last motion."

    # print "Linearizing..."
    # lin_motions1 = [linearizeMotion(m, 0.1) for m in motions1]
    # lin_motions2 = [linearizeMotion(m, 0.1) for m in motions2]

    # raw_input("-- Press anything to start --")
    # for m in lin_motions1:
    #     runConfigurationSequence(m[::-1])
    # for m in lin_motions2:
    #     runConfigurationSequence(m)

    # raw_input("-- Press anything to exit --")
