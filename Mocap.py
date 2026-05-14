
############## version 2 of Mocap system call function. It gives the position and orientation of the Zumo bot......

import sys
print(sys.executable)
sys.path.append(r'c:\users\giuseppe\appdata\local\packages\pythonsoftwarefoundation.python.3.10_qbz5n2kfra8p0\localcache\local-packages\python310\site-packages')

from TSP_natnet import NatNetClient
from scipy.spatial.transform import Rotation as R
import numpy as np
import time 
import pandas as pd
import warnings
warnings.filterwarnings("ignore") 
# print(np.__ion__)


global f_n

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged,f_n ):
    f_n = f_n+1 
    return f_n
    
def receiveRigidBodyFrame( id, position, rotation):
    return position, rotation

     

def ConnectOptitrack(localIP,serverIP,IDS):
    streamingClient = NatNetClient(localIP,serverIP,IDS)
    streamingClient.newFrameListener = receiveNewFrame
    streamingClient.rigidBodyListener = np.zeros((len(IDS), 3), dtype=object)
    streamingClient.run()

    start_time = time.time()
    print('Connecting to Opti-Track .....')
    while streamingClient.rigidBodyListener[0, 0] == 0:

        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time > 10:
            print('Did not receive data from Opti-Track')
            return False

    print('Opti-Track connected')

    return streamingClient



localIP =  "192.168.0.161" #"192.168.0.161"    # host computer
serverIP =  "192.168.0.161" #"192.168.0.161"   # client computer

IDS = [47, 52]
streamingClient = ConnectOptitrack(localIP,serverIP,IDS)





def GlobalPos(inx) :



    g_p_zumo = streamingClient.rigidBodyListener[inx,1]    #position of Zumo mocap frame with global xyz
    g_r_zumo  = streamingClient.rigidBodyListener[inx,2]    #quaternion of Zumo mocap frame with global xyz

    
        
    g_p_zumo = np.array(g_p_zumo).reshape(3,1)     # position of Zumo mocap frame with global xyz  
    g_r_zumo = R.from_quat(list(g_r_zumo)).as_matrix() # rotation of Zumo mocap frame with global xyz

    
    #print(g_r_zumo)

    # Note: Calibration matrix not needed for this setup
    # dm_r_d = np.linalg.inv([[-0.10367551 ,-0.99460742 ,-0.00273116],
    # [ 0.99357069, -0.10344109, -0.04601329],
    # [ 0.04548264, -0.00748406,  0.99893709]])  # inverse of d_r_dm

    g_r_zumo_global = g_r_zumo    # rotation of Zumo with Global frame xyz (no calibration offset)
    g_p_zumo_global = g_p_zumo    # position of Zumo wrt Global xyz
    
    

    rot_zumo_angle = R.from_matrix(g_r_zumo_global)
    angles = rot_zumo_angle.as_euler("zyx",degrees=True)

    # print('position of Zumo in global frame:', g_p_zumo_global)
    # print('Zumo is rotated by angle:  ',np.round(angles,2))

    # Combine rotation matrix and position vector
    g_T_zumo = np.vstack((np.hstack((g_r_zumo_global, g_p_zumo_global)), [0, 0, 0, 1]))  # wrt local origin

    # G_T_g = np.array([[1, 0, 0 , 0.96], [0 , 1 ,0 , 1.486], [0,0,1, -0.04], [0,0,0,1]])    
    G_T_g = np.array([[0, 1, 0 , 0], [-1 , 0 ,0 , 0], [0,0, 1, 0], [0,0,0,1]])          # Transformation matrix of Mocap origin to experimental Global origin
    
    G_T_zumo = np.matmul(G_T_g, g_T_zumo)  # wrt global origin


    # print(g_T_d)
    # for i in range(len(G_T_d)):
    #     for j in range(len(G_T_d)):
    #         if (j == 3 and i == 0) or (j == 3 and i == 1):
    #             G_T_d[i][j] = -1 * G_T_d[i][j]
    #         else:
    #             G_T_d[i][j] = G_T_d[i][j]

    # print(G_T_d)

    return G_T_zumo

print(np.round(GlobalPos(0)[:3,-1],2))


# while True:
#     for i in range(len(IDS)):
#         print('ID = ', IDS[i],'\nposition \n=', np.round(GlobalPos(i),2))
#     print("------------------------------------------------------------")
#     time.sleep(3)



# g_T_d = GlobalPos(0)

# d_T_g = np.linalg.inv(g_T_d)

# d_p_1 = np.matmul(d_T_g ,np.array([[-1.71],[-0.76],[0],[1]]))

# print(d_p_1)

    
       


    












    
    

    

     
    
    
    
    
    





