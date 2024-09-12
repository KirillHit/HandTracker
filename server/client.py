import socket, time
connect = False
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
while True:
    if not connect:
        try:
            print("try Connect")
            serv.connect(("192.168.0.15", 48569))
            print("Connect")
            connect = True
        except Exception as e:
            print(e)
    else:
        while connect:
            try:
                print("try recive")
                recv = serv.recv(1024).decode()
                print(recv)
                serv.sendall(b"42")
            except Exception as e:
                print("nooo")
                serv.close()
                serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connect = False
                print(e)