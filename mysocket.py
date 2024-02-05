
import socket
HOST = "127.0.0.1"
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    with conn:
        print('connected')
        while True:
            data = conn.recv(1024)
            print(f'data = {data}')
            conn.sendall(b"hi back")
print('finished')


import socket
HOST = "127.0.0.1"
PORT = 65432
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"hi 1")
    data=s.recv(1024)
    print(f"Received {data!r}")

    s.sendall(b"hi 2")
    data=s.recv(1024)
    print(f"Received {data!r}")

    s.sendall(b"hi 3")
    data=s.recv(1024)
    print(f"Received {data!r}")
print('client closed')