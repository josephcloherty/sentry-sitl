import socket
import time
import threading

def create_tcp_host(host="0.0.0.0", port=1234, listen=1):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Allow immediate port reuse in WSL
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(listen)
    print(f"Listening on {host}:{port}")
    return server_socket

def create_fields_string(fields_list):
    field_str = "{} " * len(fields_list)
    return field_str.format(*fields_list).rstrip()

class TCP_Relay:
    def __init__(self, num_fields=23, host="0.0.0.0", port=1234, size=1024):
        self.server_socket = create_tcp_host(host, port)
        self.num_fields = num_fields
        self.size = size
        self.message = create_fields_string([0.] * self.num_fields)
        self.linked = False
        self.message_in = None
        self.client_socket = None
        self.thread = threading.Thread(target=self._server)
        self.thread.daemon = True
        self.thread.start()

    def _server(self):
        while True:
            while self.linked:
                # 1. Non-Blocking Receive
                try:
                    # Prevents Python from freezing while waiting for Unreal Engine
                    self.client_socket.setblocking(False) 
                    self.message_in = self.client_socket.recv(self.size)
                    if self.message_in == b'':
                        raise socket.error("Client disconnected")
                except BlockingIOError:
                    # Unreal is silent, which is perfectly fine. Move on!
                    self.message_in = None
                except socket.error as e:
                    print(f"Receive error/Disconnect: {e}")
                    self.message_in = None
                    self.client_socket.close()
                    self.linked = False
                    self.client_socket = None
                    break 

                # 2. Send Telemetry Data
                if self.linked:
                    try:
                        # Append a trailing space so Unreal's "Parse Into Array" node splits it cleanly
                        payload = self.message + " "
                        self.client_socket.send(str.encode(payload))
                    except socket.error as e:
                        print(f"Send error: {e}")
                        print("Disconnected.")
                        self.client_socket.close()
                        self.linked = False
                        self.client_socket = None
                        break
                
                # Throttle to 60 FPS (0.016s) to prevent flooding Unreal Engine's TCP buffer
                time.sleep(0.016)

            if not self.linked:
                try:
                    self.client_socket, client_address = self.server_socket.accept()
                    print("Connected. Client address:", client_address)
                    self.linked = True
                except socket.error as e:
                    print(f"Accept error: {e}")

            time.sleep(0.016)
