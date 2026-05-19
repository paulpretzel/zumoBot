import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import socket
import serial
import time
import math
import sys
import csv
from collections import deque
import datetime

# --- 1. System & Comms Configuration ---
ZUMO_SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# --- 2. Kinematic Control Law Parameters ---
K_w = -2.5          # Proportional gain for turning (Flipped sign for your setup!)
MAX_V = 0.25        # Max linear speed (m/s)
AVOID_V = 0.10      # Forward speed while avoiding
AVOID_W = 3.0       # Sharp turn speed for avoidance
GOAL_TOL = 0.1      # Distance threshold to stop (m)

# --- 3. RANSAC & Camera Configuration ---
VOXEL_SIZE = 0.015         # Downsample size (1.5cm)
DISTANCE_THRESHOLD = 0.04  # RANSAC Plane Thickness
SAFE_DISTANCE = 0.50       # Stop if floor ends closer than 50cm
DEPTH_TRUNC = 2.0          # Ignore data beyond 2.0m
MIN_SCAN_DISTANCE = 0.15   # Camera blind spot

# Polar Scan Settings
THETA_MIN = -35
THETA_MAX = 35
THETA_STEP = 1
NUM_BINS = int((THETA_MAX - THETA_MIN) / THETA_STEP)
SCAN_BUFFER_SIZE = 3  

# --- Helper Functions (From your RANSAC script) ---
def detect_ground_plane(pcd):
    if not pcd.has_points():
        return None, pcd, False

    plane_model, inliers = pcd.segment_plane(distance_threshold=DISTANCE_THRESHOLD,
                                             ransac_n=3,
                                             num_iterations=250)
    if len(inliers) < 100:
        return None, pcd, False

    ground_cloud = pcd.select_by_index(inliers)
    obstacle_cloud = pcd.select_by_index(inliers, invert=True)
    return ground_cloud, obstacle_cloud, True

def extract_polar_coords(pcd):
    if not pcd.has_points():
        return np.array([]), np.array([])
    pts = np.asarray(pcd.points)
    x = pts[:, 0]
    z = np.abs(pts[:, 2]) 
    rho = np.sqrt(x ** 2 + z ** 2)
    theta = np.arctan2(x, z)
    return rho, theta

def generate_safe_driving_boundary(ground_rho, ground_theta_deg):
    boundary_scan = np.zeros(NUM_BINS)
    if len(ground_rho) == 0:
        return boundary_scan

    mask = (ground_theta_deg >= THETA_MIN) & \
           (ground_theta_deg < THETA_MAX) & \
           (ground_rho > MIN_SCAN_DISTANCE)

    valid_rho = ground_rho[mask]
    valid_angles = ground_theta_deg[mask]

    if len(valid_angles) == 0:
        return boundary_scan

    indices = ((valid_angles - THETA_MIN) / THETA_STEP).astype(int)
    indices = np.clip(indices, 0, NUM_BINS - 1)
    np.maximum.at(boundary_scan, indices, valid_rho)
    return boundary_scan

# --- Main Logic ---
def run_autonomous_zumo():
    # 1. Setup Serial
    try:
        zumo_serial = serial.Serial(ZUMO_SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[HW] Connected to Zumo on {ZUMO_SERIAL_PORT}")
    except Exception as e:
        print(f"[ERR] Serial Error: {e}")
        sys.exit(1)

    # 2. Setup UDP (Non-Blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False) 
    print(f"[HW] Listening for Mocap on port {UDP_PORT}")

    # 3. Setup Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
