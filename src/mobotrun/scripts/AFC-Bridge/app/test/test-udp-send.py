import socket

IP = "localhost"
PORT = 1669
PAYLOAD = b"Hi mom !"

print(f"<={IP} <= {PAYLOAD}")

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

sock.sendto(PAYLOAD, (IP, PORT))
