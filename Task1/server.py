from pickle import TRUE
import socket
import threading

HEADER = 50
PORT = 5050
SERVER = "localhost"
ADDR = (SERVER,PORT)
FORMAT = "utf-8"
L = '1'
R = '6'
r_intial = [0,0,0,0]
c_intial = [0,0,0,0]

rows,cols = (10,10)
map = []
total_steps = [0,0,0,0]
t=0
threads = []
conns = []
disconnect = [1,1,1,1]
flag = [0,0,0,0]
order = []
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




def handle_client(conn,i):
    m='Send Stepsize'
    m= m.encode(FORMAT)
    m_length = len(m)
    send_length = str(m_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length))
    conn.send(send_length)
    conn.send(m)
    step_length = conn.recv(HEADER).decode(FORMAT)
    step_length = int(step_length)
    step = conn.recv(step_length).decode(FORMAT)
    step = int(step)
    total_steps[i] += step
    if (total_steps[i]>=100):
        msg2 = '!DISCONNECT'
        msg2= msg2.encode(FORMAT)
        msg2_length = len(msg2)
        send_length = str(msg2_length).encode(FORMAT)
        send_length += b' '*(HEADER - len(send_length))
        conn.send(send_length)
        conn.send(msg2)
        disconnect[i]=0
        flag[i]=1
        conn.close()

    

    

def plot_grid():
    global t
    t +=1
    print(f"t = {t}")
    for i in range(4):
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
    server.listen(4)
    print("[LISTENING] clients")
    for i in range(4):
        conn,addr = server.accept()
        print(f"Worker {chr(i+65)} connected  with address {addr}")
        l = '1'
        l= l.encode(FORMAT)
        l_length = len(l)
        send_length = str(l_length).encode(FORMAT)
        send_length += b' '*(HEADER - len(send_length))
        conn.send(send_length)
        conn.send(l)
        r = '6'
        r = r.encode(FORMAT)
        r_length = len(r)
        send_length = str(r_length).encode(FORMAT)
        send_length += b' '*(HEADER - len(send_length))
        conn.send(send_length)
        conn.send(r)
        t1 = threading.Thread(target=handle_client, args=(conn,i))
        conns.append(conn)
        threads.append(t1)
        t1.start()
    connection = True
    while connection:
        for i in range(4):
            if(disconnect[i] == 1 or flag[i]==1):
                threads[i].join()

        plot_grid()
        for i in range(4):
            if(flag[i]==1):
                order.append(i)
                msg = f"Worker {chr(i+65)} escaped!"
                msg = msg.encode(FORMAT)
                msg_length = len(msg)
                send_length = str(msg_length).encode(FORMAT)
                send_length += b' '*(HEADER - len(send_length))

                for j in range(4):
                    if disconnect[j]==1:
                        conns[j].send(send_length)
                        conns[j].send(msg)
            flag[i]=0
        if(disconnect[0]==0 and disconnect[1]==0 and disconnect[2]==0 and disconnect[3]==0):
            connection = False
            break
        for i in range(4):
            if(disconnect[i] == 1):
                t2 =threading.Thread(target=handle_client, args = (conns[i],i))
                threads[i]=t2

        for i in range(4):
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
for i in range(4):
    print(f"Worker {chr(order[i]+65)}")




 








