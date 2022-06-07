from pickle import TRUE
import socket
import threading

HEADER = 50
PORT = 5050
SERVER = "localhost"
ADDR = (SERVER,PORT)
FORMAT = "utf-8"
N = 10
#=====INPUTS ARE TAKEN LATER=======================
L = 0
R = 0
K = 0
N_CLIENTS = 0
r_intial = []
c_intial = []
rows,cols = (N,N)
map = []
total_steps = []
t=0
threads = []
conns = []
disconnect = []
flag = []
order = []
#================================================
server = socket.socket()
server.bind(ADDR)

for i in range(rows):
    col = []
    for j in range(cols):
        col.append('.')
    map.append(col)

map[0][2] = map[0][7] = map[1][5] = map[2][0] = map[2][7] = map[4][4]=map[5][2]='#'
map[5][9] = map[7][1] = map[7][4] = map[8][1] = map[8][3] = map[8][9]=map[9][2]='#'
map[0][0] = 'S'
map[9][9] = 'E'



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


def handle_client(conn,i):
    m='Send Stepsize'
    send_msg(conn,m)
    step = int(receive_msg(conn))
    total_steps[i] += step
    if (total_steps[i]>=100):
        msg2 = '!DISCONNECT'
        send_msg(conn,msg2)
        disconnect[i]=0
        flag[i]=1
        conn.close()

    

    

def plot_grid():
    global t
    t +=1
    print(f"t = {t}")
    for i in range(N_CLIENTS):
        if(flag[i] == 1):
            if(r_intial[i]==9 and c_intial[i]==9):
                map[9][9]='E'
            else:
                map[r_intial[i]][c_intial[i]] = '.'
        elif(disconnect[i] == 1):
            if(r_intial[i] == 0 and c_intial[i] == 0 ):
                pass
            elif(r_intial[i] == 9 and c_intial[i] == 9 ):
                pass
            else:
                map[r_intial[i]][c_intial[i]] = '.'
            r = int(total_steps[i]/10)
            c = total_steps[i]%10
            if(map[r][c] != '#'):
                map[r][c] = chr(i+65)
            else:
                check = True
                while (check):
                    total_steps[i] -= 8
                    if(total_steps[i]<=0):
                        total_steps[i]=0
                        break
                    r = int(total_steps[i]/10)
                    c = total_steps[i]%10
                    if(map[r][c] != '#'):
                        map[r][c] = chr(i+65)
                        check = False
            if (total_steps[i] != 0 and total_steps[i] != 100):
                r_intial[i] = r
                c_intial[i] = c
        


    for i in range(rows):
       for j in range(cols):
         if(i%2==1):
            print(map[9-i][j],end="")
         else:
            print(map[9-i][9-j],end="")
       print("\n")
    


    



def start_server():
    global L
    global R
    global K
    global N
    global N_CLIENTS
#=====================Handling Main Client=====================================
    server.listen()
    print("[LISTENING] main client")
    conn_main,addr_main = server.accept()
    print(f"Main Client Connected with address {addr_main}")
    n = N
    n = str(n)
    send_msg(conn_main,n)
    
    n_clients = int(receive_msg(conn_main))
    N_CLIENTS = n_clients

  

    L = int(receive_msg(conn_main))
    R = int(receive_msg(conn_main))
    K = int(receive_msg(conn_main))

#=====================================================================================
    for i in range(N_CLIENTS):
        total_steps.append(0)
        disconnect.append(1)
        flag.append(0)
        r_intial.append(0)
        c_intial.append(0)


# =====================Listening Other Clients======================================
    
    server.listen(N_CLIENTS)
    print(f"[LISTENING] {N_CLIENTS} clients")
    for i in range(N_CLIENTS):
        conn,addr = server.accept()
        print(f"Worker {chr(i+65)} connected  with address {addr}")
        l = str(L)
        send_msg(conn,l)
        r = str(R)
        send_msg(conn,r)
        t1 = threading.Thread(target=handle_client, args=(conn,i))
        conns.append(conn)
        threads.append(t1)
        t1.start()

#===================================================================================================
    connection = True
    while connection:
        for i in range(N_CLIENTS):
            if(disconnect[i] == 1 or flag[i]==1):
                threads[i].join()

        plot_grid()
        for i in range(N_CLIENTS):
            if(flag[i]==1):
                order.append(i)
                msg = f"Worker {chr(i+65)} escaped!"
                msg = msg.encode(FORMAT)
                msg_length = len(msg)
                send_length = str(msg_length).encode(FORMAT)
                send_length += b' '*(HEADER - len(send_length))

                for j in range(N_CLIENTS):
                    if disconnect[j]==1:
                        conns[j].send(send_length)
                        conns[j].send(msg)
            flag[i]=0
        cnt = 0
        for i in range(N_CLIENTS):
            if(disconnect[i]==0):
                cnt += 1
        if(cnt == N_CLIENTS):
            connection = False
            break


        for i in range(N_CLIENTS):
            if(disconnect[i] == 1):
                t2 =threading.Thread(target=handle_client, args = (conns[i],i))
                threads[i]=t2

        for i in range(N_CLIENTS):
            if(disconnect[i] == 1):
                threads[i].start()
        






print("This is the map of our plant \n")

for i in range(rows):
    for j in range(cols):
       if(i%2==1):
           print(map[9-i][j],end="")
       else:
           print(map[9-i][9-j],end="")
    print("\n")


print(f"[STARTING] server {ADDR}")
start_server()
print("Order of workers escape is : ")
for i in range(N_CLIENTS):
    print(f"Worker {chr(order[i]+65)}")




 








