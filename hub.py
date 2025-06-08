import socket


class Hub:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect()

    def connect(self):
        self.socket.connect((self.host, self.port))

    def send_message(self, msg):
        self.socket.sendall(msg.encode())

    def close(self):
        self.socket.close()
