import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on port {UDP_PORT}...")

while True:
    try:
        data, addr = sock.recvfrom(1024)
        relative_heading = float(data.decode())
        
#         print(f"Received from {addr}: {data.decode}")
        print(f"Relative Heading: {relative_heading:.4f} rad {relative_heading * 180 / 3.14159:.2f}°")
    except Exception as e:
        print(f"Error: {e}")
