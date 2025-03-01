import socket

IP = "0.0.0.0"
PORT = 1669

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

sock.bind((IP, PORT))

while True:

    data, addr = sock.recvfrom(1024)
    print(f"=>{addr} => {str(data)}")
