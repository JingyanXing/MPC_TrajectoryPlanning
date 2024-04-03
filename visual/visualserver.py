import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import socket
import numpy as np
import pandas as pd
import math
vehicle_length = 5
vehicle_width = 1.9
delta = math.atan(vehicle_width / vehicle_length)
half_diagonal = (vehicle_width ** 2 + vehicle_length ** 2) ** 0.5 / 2


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

    data = pd.DataFrame(columns=['x', 'y', 'speed', 'headingAngle', 'wheelAngle'])
    boundryx = []
    boundry1 = []
    boundry2 = []
    boundry3 = []
    boundry_front_1 = [0] * 250
    boundry_front_2 = [3.5] * 250
    boundry_front_3 = [7] * 250
    data_index = 0

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))
    while True:
        recv_str = connection.recv(1024)
        recv_str = recv_str.decode("ascii")
        if not recv_str:
            break
        
        data.loc[len(data)] = stringToList(recv_str)
        
        #boundry
        if len(boundryx) == 0:
            boundryx.append(data.iloc[0, 0])
        else:
            while boundryx[-1] < data.iloc[-1, 0] + 25:
                boundryx.append(boundryx[-1] + 0.1)
        boundry1 = [0] * len(boundryx)
        boundry2 = [3.5] * len(boundryx)
        boundry3 = [7] * len(boundryx)


        axs[0].cla()  # 清空第一个子图
        axs[1].cla()  # 清空第二个子图

        """绘制车辆轨迹"""
        axs[0].plot(data.iloc[:, 0], data.iloc[:, 1],'-b') # 行驶轨迹
        axs[0].plot(boundryx, boundry1, color='k', label='boundary0') 
        axs[0].plot(boundryx, boundry2, color='k', label='boundary1')
        axs[0].plot(boundryx, boundry3, color='k', label='boundary2')
        axs[0].set_title('Trajectory')
        axs[0].set_xlabel('X(m)')
        axs[0].set_ylabel('y(m)')
        axs[0].legend()

        """绘制车辆姿态"""
        """以下顶点计算方法适用于小转向角"""
        boundry_front = [(data.iloc[-1, 0] - 5 + 0.1 * i) for i in range(250)]
        axs[1].plot(boundry_front, boundry_front_1, color='k') 
        axs[1].plot(boundry_front, boundry_front_2, color='k')
        axs[1].plot(boundry_front, boundry_front_3, color='k')
        dia_angle = delta + data.iloc[-1, 3]
        lower_left = (data.iloc[-1, 0] - half_diagonal * math.cos(dia_angle), 
                      data.iloc[-1, 1] - half_diagonal * math.sin(dia_angle)) # 左下顶点
        lower_right = (lower_left[0] + vehicle_length * math.cos(data.iloc[-1, 3]), 
                       lower_left[1] + vehicle_length * math.sin(data.iloc[-1, 3])) # 右下顶点
        upper_left = (lower_left[0] - vehicle_width * math.sin(data.iloc[-1, 3]), 
                      lower_left[1] + vehicle_width * math.cos(data.iloc[-1, 3]))
        upper_right = (lower_left[0] + 2 * half_diagonal * math.cos(dia_angle), 
                      lower_left[1] + 2 * half_diagonal * math.sin(dia_angle))
        rect = patches.Polygon([lower_left, lower_right, upper_right, upper_left], closed=True, edgecolor='r', facecolor='none')
        axs[1].add_patch(rect)  # 将矩形添加到当前子图中
        axs[1].set_title('Position(m)')
        axs[1].set_xlabel('X(m)')
        axs[1].set_ylabel('y(m)')
        
        plt.pause(0.01)
        plt.tight_layout()
        send_str = "Visual Server is running"
        connection.send(bytes(send_str, encoding="ascii"))
        # time.sleep(0.1)


        

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