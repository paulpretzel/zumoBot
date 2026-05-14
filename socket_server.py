import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from TSP_natnet import NatNetClient
import warnings
warnings.filterwarnings("ignore")

# Import mocap functions
from Mocap import ConnectOptitrack, GlobalPos, IDS, streamingClient

# Raspberry Pi configuration
RPI_IP = "192.168.0.113"  # RPi5 IP address
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    try:
        # Get mocap data from the Zumo (ID 52 is at index 1)
        G_T_zumo = GlobalPos(1)  # Get 4x4 transformation matrix for Zumo (ID 52)
        
        # Extract Zumo position (last column, first 3 rows)
        zumo_position = G_T_zumo[:3, 3]
        
        # Extract Zumo rotation matrix and convert to Euler angles
        zumo_rotation_matrix = G_T_zumo[:3, :3]
        zumo_rot_obj = R.from_matrix(zumo_rotation_matrix)
        zumo_angles = zumo_rot_obj.as_euler("zyx")  # in radians
        zumo_yaw = zumo_angles[0]  # Zumo heading w.r.t global axis (radians)
        
        # Get mocap data from the goal position (ID 47 is at index 0)
        G_T_goal = GlobalPos(0)  # Get 4x4 transformation matrix for goal (ID 47)
        goal_position = G_T_goal[:3, 3]
        
        # Calculate angle from Zumo to Goal w.r.t global axis
        delta_x = goal_position[0] - zumo_position[0]
        delta_y = goal_position[1] - zumo_position[1]
        angle_to_goal = np.arctan2(delta_y, delta_x)
        
        # Calculate relative heading angle (angle robot needs to turn to face goal)
        relative_heading = angle_to_goal - zumo_yaw
        # Normalize to [-π, π]
        while relative_heading > np.pi:
            relative_heading -= 2*np.pi
        while relative_heading < -np.pi:
            relative_heading += 2*np.pi
        
        # Print debug info
        print(f"Zumo Position: {np.round(zumo_position, 4)}, Goal Position: {np.round(goal_position, 4)}")
        print(f"Relative Heading: {relative_heading:.4f} rad")
        
        # Format data as: zumo_x zumo_y zumo_z zumo_yaw angle_to_goal relative_heading goal_x goal_y goal_z
        msg = f"{zumo_position[0]:.4f} {zumo_position[1]:.4f} {zumo_position[2]:.4f} {zumo_yaw:.4f} {angle_to_goal:.4f} {relative_heading:.4f} {goal_position[0]:.4f} {goal_position[1]:.4f} {goal_position[2]:.4f}"

        try:
            sock.sendto(msg.encode(), (RPI_IP, UDP_PORT))
        except Exception as send_error:
            print(f"Error sending to {RPI_IP}:{UDP_PORT} - {send_error}")
        time.sleep(0.01)  # 100 Hz
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(0.1)
