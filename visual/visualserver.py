import matplotlib.pyplot as plt
import time
import socket
import numpy as np
import pandas as pd


def VisualServer():
    def stringToList(string):
        return [float(x) for x in string.split(',')]

    # 创建一个TCP套接字
    # 这一行创建了一个套接字（socket），并将其赋值给变量 server。
    # socket.socket() 函数用于创建套接字对象，AF_INET 参数指定了使用IPv4地址族，SOCK_STREAM 参数指定了使用TCP协议。
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8888)) # 绑定到本地主机（localhost）的8888端口。可以随意更改，客户端和服务端需要使用相同的端口号。
    server.listen(5) # 监听传入的连接，参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。
    connection, address = server.accept()

    plt.figure(figsize=(10, 8))
    #pos
    plt.subplot(5, 1, 1)
    data = pd.DataFrame(columns=['x', 'y', 'speed', 'headingAngle', 'wheelAngle'])
    data_index = 0
    while True:
        recv_str = connection.recv(1024)
        recv_str = recv_str.decode("ascii")
        if not recv_str:
            break
        
        plt.clf() #清空画布上的所有内容
        data.loc[len(data)] = stringToList(recv_str)
        plt.plot(data.iloc[:, 0], data.iloc[:, 1],'-r')
        plt.pause(0.01)


        send_str = "Visual Server is running"
        connection.send(bytes(send_str, encoding="ascii"))
        time.sleep(0.1)

    connection.close()
    server.close()
    print("client end, exit!")
    exit()

VisualServer()

# plt.ion() #开启interactive mode 成功的关键函数

# t = [0]
# t_now = 0
# m = [sin(t_now)]

# for i in range(200):
#     plt.clf() #清空画布上的所有内容
#     t_now = i*0.1
#     t.append(t_now)#模拟数据增量流入，保存历史数据
#     if len(t)>10:
#         t.pop(0)#保持数据长度不超过100
#     m.append(sin(t_now))#模拟数据增量流入，保存历史数据
#     if len(m)>10:
#         m.pop(0)#保持数据长度不超过100
#     plt.plot(t,m,'-r')
#     plt.pause(0.01)