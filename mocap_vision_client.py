import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import cv2
import socket
import serial
import time
import math
import sys
import csv

# --- 1. System & Comms Configuration ---
ZUMO_SERIAL_PORT = '/dev/ttyACM0'  # RPi USB port format
BAUD_RATE = 115200

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# --- 2. Kinematic Control Law Parameters ---
K_w = 2.5           # Proportional gain for turning
MAX_V = 0.3         # Max linear speed (m/s)
AVOID_V = 0.05      # Slow forward speed while avoiding
AVOID_W = 2.0       # Sharp turn speed for avoidance
GOAL_TOL = 0.1      # Distance threshold to stop (m)

# --- 3. Camera & World Parameters ---
VOXEL_SIZE = 0.005
DISTANCE_THRESHOLD = 0.03
SAFE_DISTANCE = 0.35
DEPTH_TRUNC = 1.5

# --- 4. Filtering Parameters ---
MIN_OBSTACLE_HEIGHT = 0.08
MIN_SCAN_DISTANCE = 0.1
SOR_NEIGHBORS = 20
SOR_STD_RATIO = 2.0
MIN_CLUSTER_POINTS = 30

# --- 5. Scan Settings ---
THETA_MIN = -40
THETA_MAX = 40
THETA_STEP = 0.25
NUM_BINS = int((THETA_MAX - THETA_MIN) / THETA_STEP)

# --- Helper Functions ---
def detect_ground_plane(pcd):
    if not pcd.has_points():
        return None, pcd, False
    plane_model, inliers = pcd.segment_plane(distance_threshold=DISTANCE_THRESHOLD, ransac_n=3, num_iterations=100)
    if len(inliers) < 100:
        return None, pcd, False
    ground_cloud = pcd.select_by_index(inliers)
    obstacle_cloud = pcd.select_by_index(inliers, invert=True)
    [a, b, c, d] = plane_model
    obs_points = np.asarray(obstacle_cloud.points)
    if len(obs_points) > 0:
        dists_to_plane = np.abs(obs_points @ np.array([a, b, c]) + d)
        mask = dists_to_plane > MIN_OBSTACLE_HEIGHT
        obstacle_cloud = obstacle_cloud.select_by_index(np.where(mask)[0])
    return ground_cloud, obstacle_cloud, True

def clean_obstacle_cloud(pcd):
    if not pcd.has_points():
        return pcd
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=SOR_NEIGHBORS, std_ratio=SOR_STD_RATIO)
    pcd = pcd.select_by_index(ind)
    cl, ind = pcd.remove_radius_outlier(nb_points=5, radius=0.03)
    pcd = pcd.select_by_index(ind)
    return pcd

def generate_polar_scan(obstacle_pcd):
    scan_array = np.full(NUM_BINS, np.inf)
    if len(obstacle_pcd.points) < MIN_CLUSTER_POINTS:
        return scan_array
    pts = np.asarray(obstacle_pcd.points)
    dists = np.sqrt(pts[:, 0] ** 2 + pts[:, 2] ** 2)
    angles_rad = np.arctan2(pts[:, 0], abs(pts[:, 2]))
    angles_deg = np.degrees(angles_rad)
    mask = (angles_deg >= THETA_MIN) & (angles_deg < THETA_MAX) & (dists > MIN_SCAN_DISTANCE)
    valid_dists = dists[mask]
    valid_angles = angles_deg[mask]
    if len(valid_angles) == 0:
        return scan_array
    indices = ((valid_angles - THETA_MIN) / THETA_STEP).astype(int)
    indices = np.clip(indices, 0, NUM_BINS - 1)
    for i, idx in enumerate(indices):
        if valid_dists[i] < scan_array[idx]:
            scan_array[idx] = valid_dists[i]
    return scan_array

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
    sock.setblocking(False) # Crucial: Don't let socket delay the camera feed
    print(f"[HW] Listening for Mocap on port {UDP_PORT}")

    # 3. Setup Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    print("[HW] Starting RealSense Pipeline...")
    pipeline.start(config)

    # 4. Setup OpenCV & CSV
    cv2.namedWindow("Raw Camera", cv2.WINDOW_AUTOSIZE)
    csv_file = open('ransac_log.csv', mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Header: Timestamp, State, V_cmd, W_cmd, Bin0, Bin1... BinN
    header = ['Timestamp', 'State', 'V_Cmd', 'W_Cmd'] + [f'Bin_{i}' for i in range(NUM_BINS)]
    csv_writer.writerow(header)
    print("[HW] CSV Logging Started.")

    mocap_data = None

    try:
        while True:
            # --- A. Grab Latest Mocap Data (Flush buffer) ---
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    mocap_data = data.decode().split()
            except BlockingIOError:
                pass # Buffer is empty, move on with latest data

            # --- B. Grab & Process Camera Frame ---
            frames = pipeline.wait_for_frames()
            depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
            if not depth_frame or not color_frame: continue

            # Show Raw Video
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("Raw Camera", color_image)
            cv2.waitKey(1)

            # Process Depth
            depth_frame = temporal.process(spatial.process(decimation.process(depth_frame)))
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frame.get_data())
            pinhole = o3d.camera.PinholeCameraIntrinsic(
                intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
            temp_pcd = o3d.geometry.PointCloud.create_from_depth_image(
                o3d.geometry.Image(depth_image), pinhole, depth_scale=1000.0, depth_trunc=DEPTH_TRUNC)
            temp_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            temp_pcd = temp_pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)

            # --- C. RANSAC & Radar Analysis ---
            ground, obstacles, success = detect_ground_plane(temp_pcd)
            
            V_cmd, W_cmd = 0.0, 0.0
            state_str = "MOCAP_GOAL"
            raw_scan = np.full(NUM_BINS, np.inf)

            if success:
                obstacles = clean_obstacle_cloud(obstacles)
                raw_scan = generate_polar_scan(obstacles)
                
                # Check center slice for obstacles
                center_slice = raw_scan[int(NUM_BINS / 2 - 20):int(NUM_BINS / 2 + 20)]
                real_center_dists = center_slice[center_slice != np.inf]
                
                obstacle_in_path = len(real_center_dists) > 0 and np.min(real_center_dists) < SAFE_DISTANCE

                if obstacle_in_path:
                    state_str = "OBSTACLE_AVOID"
                    # Compare left vs right side openness
                    left_slice = raw_scan[:int(NUM_BINS/2)]
                    right_slice = raw_scan[int(NUM_BINS/2):]
                    
                    left_avg = np.mean(left_slice[left_slice != np.inf]) if len(left_slice[left_slice != np.inf]) > 0 else DEPTH_TRUNC
                    right_avg = np.mean(right_slice[right_slice != np.inf]) if len(right_slice[right_slice != np.inf]) > 0 else DEPTH_TRUNC
                    
                    V_cmd = AVOID_V
                    # If left is more open, turn left (+W). Else turn right (-W).
                    W_cmd = AVOID_W if left_avg > right_avg else -AVOID_W

            # --- D. Mocap Go-To-Goal Logic (If Path is Clear) ---
            if state_str == "MOCAP_GOAL" and mocap_data:
                zumo_x, zumo_y = float(mocap_data[0]), float(mocap_data[1])
                relative_heading = float(mocap_data[5])
                goal_x, goal_y = float(mocap_data[6]), float(mocap_data[7])
                
                distance = math.sqrt((goal_x - zumo_x)**2 + (goal_y - zumo_y)**2)
                
                if distance < GOAL_TOL:
                    state_str = "GOAL_REACHED"
                    V_cmd, W_cmd = 0.0, 0.0
                else:
                    W_cmd = K_w * relative_heading
                    speed_scale = max(0.0, 1.0 - (abs(relative_heading) / (math.pi / 2)))
                    V_cmd = MAX_V * speed_scale

            # --- E. Execute & Log ---
            command = f"{V_cmd:.3f},{W_cmd:.3f}\n"
            zumo_serial.write(command.encode('utf-8'))
            
            # Write to CSV
            csv_row = [time.time(), state_str, round(V_cmd,3), round(W_cmd,3)] + [round(x, 3) if x != np.inf else 1.5 for x in raw_scan]
            csv_writer.writerow(csv_row)

            print(f"[{state_str}] V: {V_cmd:.2f} | W: {W_cmd:.2f}")

    except KeyboardInterrupt:
        print("\n[SYS] Shutting down safely...")
    finally:
        pipeline.stop()
        command = "0.0,0.0\n"
        zumo_serial.write(command.encode('utf-8'))
        zumo_serial.close()
        csv_file.close()
        cv2.destroyAllWindows()
        print("[SYS] Resources released.")

if __name__ == "__main__":
    run_autonomous_zumo()
