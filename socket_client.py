import socket
import sys

# Socket configuration
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"[RPi Client] Listening on port {UDP_PORT}...")
print(f"[RPi Client] Waiting for data from Windows server...")
print(f"[RPi Client] Expected data format: zumo_x zumo_y zumo_z zumo_yaw angle_to_goal relative_heading goal_x goal_y goal_z")
sys.stdout.flush()

while True:
    try:
        # Receive data
        data, addr = sock.recvfrom(1024)
        values = data.decode().split()
        
        # Parse the 9 values
        zumo_x = float(values[0])
        zumo_y = float(values[1])
        zumo_z = float(values[2])
        zumo_yaw = float(values[3])
        angle_to_goal = float(values[4])
        relative_heading = float(values[5])
        goal_x = float(values[6])
        goal_y = float(values[7])
        goal_z = float(values[8])
        
        # Convert angles to degrees for display
        zumo_yaw_deg = zumo_yaw * 180 / 3.14159
        angle_to_goal_deg = angle_to_goal * 180 / 3.14159
        relative_heading_deg = relative_heading * 180 / 3.14159
        
        print(f"\n[RPi Client] Received from {addr[0]}:{addr[1]}")
        print(f"  Zumo Position: ({zumo_x:.4f}, {zumo_y:.4f}, {zumo_z:.4f})")
        print(f"  Zumo Yaw: {zumo_yaw:.4f} rad ({zumo_yaw_deg:.2f}°)")
        print(f"  Angle to Goal: {angle_to_goal:.4f} rad ({angle_to_goal_deg:.2f}°)")
        print(f"  Relative Heading: {relative_heading:.4f} rad ({relative_heading_deg:.2f}°)")
        print(f"  Goal Position: ({goal_x:.4f}, {goal_y:.4f}, {goal_z:.4f})")
        sys.stdout.flush()
        
    except ValueError as ve:
        print(f"[RPi Client] Error: Invalid data format - {ve}")
    except Exception as e:
        print(f"[RPi Client] Error: {e}")
