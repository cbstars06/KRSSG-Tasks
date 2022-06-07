import socket


HEADER = 50
SERVER = "localhost"
PORT = 5050
ADDR = (SERVER,PORT)
FORMAT = "utf-8"

main_client = socket.socket()
main_client.connect(ADDR)


def send_msg(conn,msg):
    msg = msg.encode(FORMAT)
    msg_length = len(msg)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length))
    conn.send(send_length)
    conn.send(msg)

def receive_msg(conn):
    msg_length = conn.recv(HEADER).decode(FORMAT)
    msg_length = int(msg_length)
    msg = conn.recv(msg_length).decode(FORMAT)
    return msg    


n = int(receive_msg(main_client))


n_clients = input("Enter no. of clients: ")


send_msg(main_client,n_clients)

l = input(f"Enter value of L between {1} and {n}: ")
r = input(f"Enter value of R between {l} and {n}: ")
k = input(f"Enter value of K between {1} and {2*n}: ")

send_msg(main_client,l)
send_msg(main_client,r)
send_msg(main_client,k)






