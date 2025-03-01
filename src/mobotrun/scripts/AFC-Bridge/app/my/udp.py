import socket

class MyInternalCommunication():

    def __init__(self):

        self.SOCK = None
        self.HOST = None
        self.PORT = None

    def my_init(self, host:str, port:int):

        self.HOST = host
        self.PORT = port
        self.SOCK = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def my_send(self, payload):

        self.SOCK.sendto(payload, (self.HOST, self.PORT))
