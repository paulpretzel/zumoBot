import pyrealsense2 as rs
import numpy as np
import cv2  # Restored OpenCV for the camera view
import socket
import serial
import math
import sys
import csv
import datetime
from collections import deque

# --- 1. System & Comms Configuration ---
ZUMO_SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# --- 2. Kinematic Control Law Parameters ---
K_w = -2.5          # Proportional gain for turning
MAX_V = 0.25        # Max linear speed (m/s)
AVOID_V = 0.10      # Forward speed while avoiding
AVOID_W = 3.0       # Sharp turn speed for avoidance
GOAL_TOL = 0.1      # Distance threshold to stop (m)

# --- 3. Camera & RANSAC Parameters ---
SUBSAMPLE_RATE = 10        # Downsample depth image to save Pi CPU
DISTANCE_THRESHOLD = 0.04  # RANSAC floor thickness (4cm)
SAFE_DISTANCE = 0.60       # Look ahead distance for obstacles
DEPTH_TRUNC = 2.0          # Max depth distance
MIN_SCAN_DISTANCE = 0.15   # Ignore the Zumo's own scoop

# --- 4. Polar Scan Settings ---
THETA_MIN = -35
THETA_MAX = 35
THETA_STEP = 1
NUM_BINS = int((THETA_MAX - THETA_MIN) / THETA_STEP)
SCAN_BUFFER_SIZE = 3

# --- 5. Pure NumPy RANSAC Implementation ---
def detect_ground_plane_numpy(points):
    """Custom RANSAC to replace Open3D. Returns Ground Points, Obstacle Points."""
    if len(points) < 100:
        return points, points, False

    best_inliers_mask = None
    max_inliers = 0

    # Fast NumPy RANSAC Loop
    for _ in range(100):
        idx = np.random.choice(len(points), 3, replace=False)
        p1, p2, p3 = points[idx]

        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm == 0: continue
        normal = normal / norm

        d = -np.dot(normal, p1)
        distances = np.abs(np.dot(points, normal) + d)
        inliers_mask = distances < DISTANCE_THRESHOLD
        num_inliers = np.sum(inliers_mask)

        if num_inliers > max_inliers:
            max_inliers = num_inliers
            best_inliers_mask = inliers_mask

    if max_inliers < 100:
        return points, points, False

    ground_points = points[best_inliers_mask]
    obstacle_points = points[~best_inliers_mask]
    return ground_points, obstacle_points, True

def extract_polar_coords_numpy(points):
    if len(points) == 0:
        return np.array([]), np.array([])
    x = points[:, 0]
    z = np.abs(points[:, 2])  
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
    # Setup Serial
    try:
        zumo_serial = serial.Serial(ZUMO_SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[HW] Connected to Zumo on {ZUMO_SERIAL_PORT}")
    except Exception as e:
        print(f"[ERR] Serial Error: {e}")
        sys.exit(1)

    # Setup UDP (Non-Blocking)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False) 
    print(f"[HW] Listening for Mocap on port {UDP_PORT}")

    # Setup Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # Restored Color Stream
    
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    print("[HW] Starting RealSense Pipeline...")
    profile = pipeline.start(config)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    # Restored OpenCV Window
    cv2.namedWindow("Zumo Vision (Raw)", cv2.WINDOW_AUTOSIZE)

    # Setup CSV Logging
    csv_file = open('ransac_nav_log.csv', mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ["Timestamp", "State", "V_Cmd", "W_Cmd"] + [f"{THETA_MIN + (i * THETA_STEP)}deg" for i in range(NUM_BINS)]
    csv_writer.writerow(header)

    scan_buffer = deque(maxlen=SCAN_BUFFER_SIZE)
    mocap_data = None
    
    # Anti-Chattering Lock
    avoid_direction_locked = 0.0 

    try:
        while True:
            # A. Fetch Mocap Data
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    mocap_data = data.decode().split()
            except BlockingIOError:
                pass 

            # B. Fetch Camera Frames
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame: continue

            # Render OpenCV Camera View
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("Zumo Vision (Raw)", color_image)
            
            # Press 'q' in the video window to quit gracefully
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[SYS] Quit command received via camera window.")
                break

            # Filter Depth
            depth_frame = temporal.process(spatial.process(decimation.process(depth_frame)))
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # C. Fast NumPy Point Cloud Generation
            depth_sub = depth_image[::SUBSAMPLE_RATE, ::SUBSAMPLE_RATE]
            z = depth_sub * depth_scale
            
            valid = (z > 0) & (z < DEPTH_TRUNC)
            v_sub, u_sub = np.where(valid)
            z_valid = z[valid]
            
            u_orig = u_sub * SUBSAMPLE_RATE
            v_orig = v_sub * SUBSAMPLE_RATE
            
            x = (u_orig - intrinsics.ppx) * z_valid / intrinsics.fx
            y = (v_orig - intrinsics.ppy) * z_valid / intrinsics.fy
            
            # Fixed Array Concatenation Bug
            points = np.column_stack((x, -y, -z_valid))

            # D. Custom RANSAC Floor Detection
            ground, obstacles, success = detect_ground_plane_numpy(points)
            
            V_cmd, W_cmd = 0.0, 0.0
            state_str = "MOCAP_GOAL"
            filtered_scan = np.zeros(NUM_BINS)

            if success:
                ground_rho, ground_theta = extract_polar_coords_numpy(ground)
                ground_theta_deg = np.degrees(ground_theta)
                raw_scan = generate_safe_driving_boundary(ground_rho, ground_theta_deg)

                window = np.ones(3) / 3.0
                raw_scan = np.convolve(raw_scan, window, mode='same')
                scan_buffer.append(raw_scan)
                filtered_scan = np.median(np.array(scan_buffer), axis=0)

                # Avoidance Logic with Hysteresis
                center_slice = filtered_scan[30:40] 
                valid_floor = center_slice[center_slice > 0]

                if len(valid_floor) > 0:
                    floor_edge_dist = np.min(valid_floor)
                    if floor_edge_dist < SAFE_DISTANCE:
                        state_str = "OBSTACLE_AVOID"
                        
                        # Lock steering direction to prevent chattering
                        if avoid_direction_locked == 0.0:
                            left_space = np.mean(filtered_scan[:35])
                            right_space = np.mean(filtered_scan[35:])
                            avoid_direction_locked = AVOID_W if left_space > right_space else -AVOID_W
                        
                        V_cmd = AVOID_V
                        W_cmd = avoid_direction_locked
                    else:
                        # Path is clear, unlock steering
                        avoid_direction_locked = 0.0
                else:
                    # No floor detected ahead
                    state_str = "OBSTACLE_AVOID"
                    if avoid_direction_locked == 0.0:
                        avoid_direction_locked = AVOID_W 
                    V_cmd = -0.05
                    W_cmd = avoid_direction_locked 

            # E. Mocap Logic 
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

            # F. Execute & Log
            command = f"{V_cmd:.3f},{W_cmd:.3f}\n"
            zumo_serial.write(command.encode('utf-8'))
            
            current_time = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            csv_row = [current_time, state_str, round(V_cmd,3), round(W_cmd,3)] + [round(val, 4) for val in filtered_scan]
            csv_writer.writerow(csv_row)

            print(f"[{state_str}] V: {V_cmd:.2f} W: {W_cmd:.2f}")

    except KeyboardInterrupt:
        print("\n[SYS] Shutting down...")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()  # Cleanly close the camera view
        zumo_serial.write(b"0.0,0.0\n")
        zumo_serial.close()
        csv_file.close()
        print("[SYS] Resources released.")

if __name__ == "__main__":
    run_autonomous_zumo()
