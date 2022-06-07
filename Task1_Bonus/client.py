import socket
import random


HEADER = 50
SERVER = "localhost"
PORT = 5050
ADDR = (SERVER,PORT)
FORMAT = "utf-8"
message = ' '
call = 0

client = socket.socket()
client.connect(ADDR)


l_length = client.recv(HEADER).decode(FORMAT)
l_length = int(l_length)
l = client.recv(l_length).decode(FORMAT)
l=int(l)
r_length = client.recv(HEADER).decode(FORMAT)
r_length = int(r_length)
r = client.recv(r_length).decode(FORMAT)
r=int(r)


def receive_msg():
    global message
    global call
    message_length = client.recv(HEADER).decode(FORMAT)
    message_length = int(message_length)
    message = client.recv(message_length).decode(FORMAT)
    if(message =="Send Stepsize"):
        call=1
    elif(message == "!DISCONNECT"):
        print(message)
        call = 2
    else:
        print(message)


def send_msg():
        n = random.randint(l,r)
        n = str(n).encode(FORMAT)
        n_length = len(n)
        send_length = str(n_length).encode(FORMAT)
        send_length += b' '*(HEADER - len(send_length))
        client.send(send_length)
        client.send(n)

def loop_start():
    global call
    connected = True
    
    while connected:
        receive_msg()
        if(call==1):
            send_msg()
            call=0
        elif(call==2):
            connected=False
        elif(call==0):
            continue
        

        

loop_start()


    



