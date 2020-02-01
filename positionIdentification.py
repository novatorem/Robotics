# Before running this script make sure you have loaded the scene "positionIdentification.ttt" in V-REP!
#
# This program was written for python2.7
#
# The script depends on:
#
# b0RemoteApi (Python script), which depends on:
# msgpack (Python messagePack binding, install with "pip install msgpack")
# b0.py (Python script), which depends on:
# b0 (shared library), which depends on:
# boost_chrono (shared library)
# boost_system (shared library)
# boost_thread (shared library)
# libzmq (shared library)

import math
import numpy as np
import b0RemoteApi
np.seterr(divide='ignore', invalid='ignore')

allProd = []
allSValues = []
allRotations = []
allPositions = []

def convert(s, angle):
    # define helper matrices
    idn = np.identity(3)
    sw = np.matrix([[0,            -(s.item(2)), (s.item(1))],
                    [(s.item(2)),  0,            -(s.item(0))],
                    [-(s.item(1)), (s.item(0)),  0]])

    sv = np.matrix([[s.item(3)],
                    [s.item(4)],
                    [s.item(5)]])

    # calculate rotational component
    rot = idn + (math.sin(angle) * sw) + ((1 - math.cos(angle)) * (sw * sw))
    global allRotations
    allRotations.append(rot)
    # calculate positional component

    posL = (idn * angle) + ((1 - math.cos(angle)) * sw)
    posR = (angle - math.sin(angle)) * (sw * sw)
    pos = (posL + posR) * sv
    global allPositions
    allPositions.append(pos)

    # combine both into matrix
    finalMatrix = np.matrix([[rot.item(0), rot.item(1), rot.item(2), pos.item(0)],
                             [rot.item(3), rot.item(4), rot.item(5), pos.item(1)],
                             [rot.item(6), rot.item(7), rot.item(8), pos.item(2)],
                             [0,           0,           0,           1]])
    return(finalMatrix)

# This function takes a list containing actuation angles and returns a matrix containing the end-effector pose
def forwardKinematicsPEF(angles):

    # define end affector frame

    M = np.matrix([[1, 0, 0, (-(2 * (3**0.5 / 2.0) * 42.78) + 7) / 1000.0],
                   [0, 1, 0, 0],
                   [0, 0, 1, (275.5 + 290 + 123.3 + 2*42.78 + 2*(1 / 2.0) * 42.78 + 160) / 1000.0],
                   [0, 0, 0, 1]])

    # define screw axes
    w = [0, 0, 0, 0, 0, 0]
    q = [0, 0, 0, 0, 0, 0]
    v = [0, 0, 0, 0, 0, 0]
    s = [0, 0, 0, 0, 0, 0]

    w[0] = np.transpose(np.matrix([0,0,-1]))
    q[0] = np.transpose(np.matrix([0,0,0]))
    v[0] = np.cross(np.negative(w[0]),q[0],axis=0)
    s[0] = np.concatenate((w[0],v[0]),axis=0)

    w[1] = np.transpose(np.matrix([1,0,0]))
    q[1] = np.transpose(np.matrix([0,0,275.5 / 1000.0]))
    v[1] = np.cross(np.negative(w[1]),q[1],axis=0)
    s[1] = np.concatenate((w[1],v[1]),axis=0)

    w[2] = np.transpose(np.matrix([-1,0,0]))
    q[2] = np.transpose(np.matrix([0,0,(275.5 + 290)/1000.0]))
    v[2] = np.cross(np.negative(w[2]),q[2],axis=0)
    s[2] = np.concatenate((w[2],v[2]),axis=0)

    w[3] = np.transpose(np.matrix([0,0,-1]))
    q[3] = np.transpose(np.matrix([7/1000.0,0,0]))
    v[3] = np.cross(np.negative(w[3]),q[3],axis=0)
    s[3] = np.concatenate((w[3],v[3]),axis=0)

    w[4] = np.transpose(np.matrix([math.cos(math.radians(30)), 0, -math.sin(math.radians(30))]))
    q[4] = np.transpose(np.matrix([7/1000, 0, (275.5 + 290 + 123.3 + 42.78) / 1000.0]))
    v[4] = np.cross(np.negative(w[4]),q[4],axis=0)
    s[4] = np.concatenate((w[4],v[4]),axis=0)

    w[5] = np.transpose(np.matrix([0,0,-1]))
    q[5] = np.transpose(np.matrix([(-(2 * (3**0.5 / 2.0) * 42.78) + 7)/1000.0, 0, 0]))
    v[5] = np.cross(np.negative(w[5]),q[5],axis=0)
    s[5] = np.concatenate((w[5],v[5]),axis=0)

    global allSValues
    allSValues = s

    # compute the product of exponentials
    prod = convert(s[0], angles[0])
    global allProd
    allProd.append(prod)
    for i in range(5):
        prod = np.matmul(prod, convert(s[i+1], angles[i+1]))
        allProd.append(prod)

    pose = prod * M
    return(pose)

def denavitHartenberg(lst):
    linkTwist  = lst[0] #alpha
    linkLength = lst[1] #a
    linkOffset = lst[2] #d
    jointAngle = lst[3] #theta

    M = np.matrix([
    
    [math.cos((jointAngle)),                                     -math.sin((jointAngle)),                                    0,                                  linkLength],
    [math.sin((jointAngle)) * math.cos((linkTwist)), math.cos((jointAngle)) * math.cos((linkTwist)), -math.sin((linkTwist)), -(linkOffset * math.sin((linkTwist)))],
    [math.sin((jointAngle)) * math.sin((linkTwist)), math.cos((jointAngle)) * math.sin((linkTwist)), math.cos((linkTwist)),  linkOffset * math.cos((linkTwist))],
    [0,                                                                      0,                                                                      0,                                  1]])

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

    pose = denavitHartenberg(allValues[0])
    for i in range(6):
        pose *= denavitHartenberg(allValues[i + 1])

    return pose

def Ad(matrix):

    rot = np.matrix([[matrix.item(0), matrix.item(1), matrix.item(2)],
                     [matrix.item(4), matrix.item(5), matrix.item(6)],
                     [matrix.item(8), matrix.item(9), matrix.item(10)]])
    pos = np.matrix([[matrix.item(3), matrix.item(7), matrix.item(11)]])

    posBracket = np.matrix([[0, -(pos.item(2)), (pos.item(1))],
                            [(pos.item(2)), 0, -(pos.item(0))],
                            [-(pos.item(1)), (pos.item(0)),  0]])

    return(np.vstack((np.hstack((rot,               np.zeros((3,3)))),
                      np.hstack((posBracket * rot,  rot)))))

# This function takes a list containing actuation angles and returns the jacobian matrix
def velocityKinematics(angles):

    #First jacobian is simply [s1][v1]
    jacobian = allSValues[0]

    for i in range(0, 5):
        jacobian = np.hstack((jacobian, (np.matmul(Ad(allProd[i]), allSValues[i + 1]))))

    return jacobian

# This function takes the jacobian matrix and returns the reciprocal condition number
def calcRCond(jacobian):
    _, ev, _ = np.linalg.svd(jacobian)
    mx = ev[0]
    mn = ev[-1]
    rcond = mn/mx
    return rcond

# This function takes the jacobian matrix and returns the singular vectors and values associated with the end-effector position\
# Note: You are allowed to use the SVD method from numpy!
def calcSingularValues(jacobian):
    
    u, s, _ = np.linalg.svd(jacobian[3:])

    u_1 = np.matrix([u.item(0, 0), u.item(1, 0), u.item(2, 0)])
    u_2 = np.matrix([u.item(0, 1), u.item(1, 1), u.item(2, 1)])
    u_3 = np.matrix([u.item(0, 2), u.item(1, 2), u.item(2, 2)])

    sigma_1 = s[0]
    sigma_2 = s[1]
    sigma_3 = s[2]
    
    return u_1, u_2, u_3, sigma_1, sigma_2, sigma_3

# given two points in [x, y, z] form, returns euclidean distance betwene them
def dist(a, b=[0,0,0]):
    return (sum([(a[i] - b[i])**2 for i in range(3)]))**0.5

#This function takes the three singular vectors and values associated with the end-effector position and draws the manipulability
#ellipsoid. The handles of the drawn objects should be returned to delete them later in the code, when the scene is reset.
def drawManEllipsoid(u_1,u_2,u_3,sigma_1,sigma_2,sigma_3,ee_pose):
    
    list_handles = []

    # get end positions from three axis vectors
    norm = [(sigma_1 * u_1).tolist()[0], (sigma_2 * u_2).tolist()[0], (sigma_3 * u_3).tolist()[0]]
    ee_pos = [ee_pose.item(i, 3) for i in range(3)]
    pos = [[n[i] + ee_pos[i] for i in range(3)] for n in norm]

    for p in pos:
        _, newSphere = client.simxAddDrawingObject_spheres(0.02, [0,225,0], p, client.simxServiceCall())
        list_handles.append(newSphere)

    granularity = 30
    max_dist = [math.sqrt(sum([(p[i] - ee_pos[i])**2 for i in range(3)])) for p in pos]

    # draw the frame
    for i in range(3):
        for j in range(granularity):
            # compute points from EE to pos to draw a "line"
            coords = [ee_pos[k] + (norm[i][k] * float(j)/granularity) for k in range(3)]
            curr_dist = math.sqrt(sum([(coords[i] - ee_pos[k])**2 for k in range(3)]))
            if curr_dist > max_dist:
                break
            _, newSphere = client.simxAddDrawingObject_spheres(0.01, [200,125,0], coords, client.simxServiceCall())
            list_handles.append(newSphere)

    # draw the ellipsoid

    # start by iterating through the "z" values
    z_start = [ee_pos[i] - norm[2][i] for i in range(3)]
    for z in range(granularity):
        # get current z value
        z_current = [z_start[i] + (norm[2][i] * 2*z/granularity) for i in range(3)]

        # get dist from z to ee
        z_dist = dist(z_current, ee_pos)

        # calculate x norm at current z
        z_ratio = 0 if z_dist == 0 else (z_dist**2 / dist(norm[2])**2) # fix rounding error
        z_ratio = z_ratio if z_ratio < 1 else 1 # fix rounding error
        x_norm_ratio = ((1 - z_ratio) * dist(norm[0]))**0.5 / (dist(norm[0]))**0.5
        x_norm = [norm[0][i] * x_norm_ratio for i in range(3)]

        # calculate x start point
        x_start = [z_current[i] - x_norm[i] for i in range(3)]

        
        for x in range(granularity):

            # get current x value
            x_current = [x_start[i] + (2 * x_norm[i] * x / (granularity - 1)) for i in range(3)]

            # get dist from x to z axis
            x_dist = dist(x_current, z_current)

            # calculate y norm at curren x, z
            x_ratio = (x_dist**2 / dist(norm[0])**2) if x_dist != 0 else 0
            combined_ratio = z_ratio + x_ratio
            combined_ratio = combined_ratio if combined_ratio < 1 else 1 # fix rounding error

            y_norm_ratio = 0 if dist(norm[1]) == 0 else ((1 - combined_ratio) * dist(norm[1]))**0.5 / (dist(norm[1]))**0.5 # rounding
            y_norm = [norm[1][i] * y_norm_ratio for i in range(3)]

            # get and plot final points
            y1 = [x_current[i] + y_norm[i] for i in range(3)]
            y2 = [x_current[i] - y_norm[i] for i in range(3)]
            _, newSphere = client.simxAddDrawingObject_spheres(0.01, [125,125,225], y1, client.simxServiceCall())
            list_handles.append(newSphere)
            _, newSphere = client.simxAddDrawingObject_spheres(0.01, [125,125,225], y2, client.simxServiceCall())
            list_handles.append(newSphere)
            
    return list_handles

with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi') as client:    
    doNextStep=True
    
    #Create callback functions for certain events
    def simulationStepStarted(msg):
        simTime=msg[1][b'simulationTime']
        print('Simulation step started. Simulation time: ',simTime)
        
    def simulationStepDone(msg):
        simTime=msg[1][b'simulationTime']
        print('Simulation step done. Simulation time: ',simTime)
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
    
    #Choose your angles here!
    #Define desired joint angles in rad
    # 1

    angle1 = 0*math.pi/180
    angle2 = 0*math.pi/180
    angle3 = 0*math.pi/180
    angle4 = 0*math.pi/180
    angle5 = 0*math.pi/180
    angle6 = 0*math.pi/180
    
    # 2

    # angle1 = 0*math.pi/180
    # angle2 = 90*math.pi/180
    # angle3 = 90*math.pi/180
    # angle4 = 0*math.pi/180
    # angle5 = 90*math.pi/180
    # angle6 = 0*math.pi/180
    
    # 3

    # angle1 = 45*math.pi/180
    # angle2 = -75*math.pi/180
    # angle3 = -90*math.pi/180
    # angle4 = 90*math.pi/180
    # angle5 = 45*math.pi/180
    # angle6 = 90*math.pi/180
    
    # 4

    # angle1 = -90*math.pi/180
    # angle2 = -30*math.pi/180
    # angle3 = 90*math.pi/180
    # angle4 = 90*math.pi/180
    # angle5 = 0*math.pi/180
    # angle6 = 45*math.pi/180
    
    # 5

    # angle1 = -60*math.pi/180
    # angle2 = 30*math.pi/180
    # angle3 = -45*math.pi/180
    # angle4 = 35*math.pi/180
    # angle5 = -100*math.pi/180
    # angle6 = -120*math.pi/180
    
    # 6

    # angle1 = 0*math.pi/180
    # angle2 = -90*math.pi/180
    # angle3 = -90*math.pi/180
    # angle4 = 0*math.pi/180
    # angle5 = -90*math.pi/180
    # angle6 = 0*math.pi/180
    
    # 7

    # angle1 = 30*math.pi/180
    # angle2 = -45*math.pi/180
    # angle3 = -60*math.pi/180
    # angle4 = 60*math.pi/180
    # angle5 = 30*math.pi/180
    # angle6 = 60*math.pi/180
    
    # 8

    # angle1 = 0*math.pi/180
    # angle2 = 0*math.pi/180
    # angle3 = -45*math.pi/180
    # angle4 = 45*math.pi/180
    # angle5 = 90*math.pi/180
    # angle6 = 90*math.pi/180
    
    # 9

    # angle1 = -45*math.pi/180
    # angle2 = -90*math.pi/180
    # angle3 = -90*math.pi/180
    # angle4 = 0*math.pi/180
    # angle5 = -90*math.pi/180
    # angle6 = -90*math.pi/180
    
    # 10

    # angle1 = -30*math.pi/180
    # angle2 = 0*math.pi/180
    # angle3 = 60*math.pi/180
    # angle4 = -30*math.pi/180
    # angle5 = 120*math.pi/180
    # angle6 = -90*math.pi/180

    print "\nAngles:", angle1/math.pi*180, angle2/math.pi*180, angle3/math.pi*180, angle4/math.pi*180, angle5/math.pi*180, angle6/math.pi*180
    
    #Calculate forward kinematics based on implemented functions (these functions needs to be implemented!!)
    eePoseFK_PEF = forwardKinematicsPEF([angle1,angle2,angle3,angle4,angle5,angle6])
    eePoseFK_DH = forwardKinematicsDH([angle1,angle2,angle3,angle4,angle5,angle6])

    print "\nPEF Pose:\n"
    print eePoseFK_PEF
    print "\nDH Pose:\n"
    print eePoseFK_DH
    
    #Get end-effector position from pose
    eePositionFK1 = [eePoseFK_PEF[0,3],eePoseFK_PEF[1,3],eePoseFK_PEF[2,3]]
    eePositionFK = [eePoseFK_DH[0,3],eePoseFK_DH[1,3],eePoseFK_DH[2,3]]
    
    #Draw calculated end-effector position as a blue sphere in V-REP
    error, visEEHandle = client.simxAddDrawingObject_spheres(0.025,[0,0,125],[eePositionFK[0],eePositionFK[1],eePositionFK[2]],client.simxServiceCall())
    
    #Set angles as targets for joint controllers (joint offsets are considered to match the V-REP home position)
    client.simxSetJointPosition(joint1Handle,angle1 - math.pi/2,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint2Handle,angle2 + math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint3Handle,angle3 + math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint4Handle,angle4,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint5Handle,angle5,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint6Handle,angle6,client.simxDefaultPublisher())
    
    #Wait for user input
    raw_input('Press Enter to continue...')
    
    #Get end-effector pose from V-REP 
    error,eePose=client.simxGetObjectPose(eeHandle,-1,client.simxServiceCall())
    
    #Transform obtained pose in 4x4 frame instead of position with quaternion
    s = 1/(eePose[3]*eePose[3] + eePose[4]*eePose[4] + eePose[5]*eePose[5] + eePose[6]*eePose[6])
    qr = eePose[6]
    qi = eePose[3]
    qj = eePose[4]
    qk = eePose[5]
    
    eePose = np.matrix([[1 - 2*s*(qj*qj + qk*qk), 2*s*(qi*qj - qk*qr),     2*s*(qi*qk + qj*qr),     eePose[0]],
              [2*s*(qi*qj + qk*qr),     1 - 2*s*(qi*qi + qk*qk), 2*s*(qj*qk - qi*qr),     eePose[1]],
              [2*s*(qi*qk - qj*qr),     2*s*(qj*qk + qi*qr),     1 - 2*s*(qi*qi + qj*qj), eePose[2]],
              [0,                       0,                       0,                       1]])
    
    print "\nVREP Pose:\n"
    print eePose
    
    
    #Calculate velocity kinematics based on implemented function (this function needs to be implemented!!)
    jacobian = velocityKinematics([angle1,angle2,angle3,angle4,angle5,angle6])
    
    #Calculate reciprocal condition number for the obtained Jacobian matrix here (this function needs to be implemented!!)
    rcond = calcRCond(jacobian)
    print "\nRcond:", rcond, "\n"
    
    #Visualize manipulability ellipsoid w.r.t. end-effector position (not pose!) in the V-REP scene
    #In order to do so call the predefined function drawManEllipsoid with the three singular vectors and values calculated for the end-effector position
    #(This function needs to be implemented!)
    u_1,u_2,u_3,sigma_1,sigma_2,sigma_3 = calcSingularValues(jacobian)
    print "u_1:", u_1
    print "u_2:", u_2
    print "u_3:", u_3
    print "sigma_1:", sigma_1
    print "sigma_2:", sigma_2
    print "sigma_3:", sigma_3
    
    drawHandles = drawManEllipsoid(u_1,u_2,u_3,sigma_1,sigma_2,sigma_3,eePose)
    
    #Wait for user input
    raw_input('Press Enter to continue...')
    
    #Remove ellipsoid
    for i in drawHandles:
        client.simxRemoveDrawingObject(i, client.simxDefaultPublisher())
    
    #Remove drawn calculated end-effector position
    client.simxRemoveDrawingObject(visEEHandle,client.simxDefaultPublisher())
    
    #Reset joint angles to initial values
    client.simxSetJointPosition(joint1Handle,-math.pi/2,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint2Handle,math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint3Handle,math.pi,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint4Handle,0,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint5Handle,0,client.simxDefaultPublisher())
    client.simxSetJointPosition(joint6Handle,0,client.simxDefaultPublisher())