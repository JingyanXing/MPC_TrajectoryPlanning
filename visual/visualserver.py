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


def stringToList(string):
        return [float(x) for x in string.split(',')]


# 当接收到的数据长度超出设定值，会导致数据被切割产生错误，所以单独写一个接收函数
def receive_messages(connection):
    buffer = ""
    msg_list = []
    msg_length = 8000
    while True:
        data = connection.recv(msg_length).decode("ascii")
        buffer += data
        while "\n" in buffer:
            message, buffer = buffer.split("\n", 1)
            msg_list.append(stringToList(message))
        # 当所有数据被读取完成后，会卡在data等待新的数据进入，造成循环无法退出，
        if data[-1] == '*':
            break
    return msg_list


def VisualServer():
    # 可视化模式选择
    visual_mode = input("Please input the visual mode(0:FullMsgVisual, 1:PlanningTrajectoryVisual): ")
    while visual_mode != '0' and visual_mode != '1':
        print("Error with visual mode, please input again.")
        visual_mode = input("Please input the visual mode(0:FullMsgVisual, 1:PlanningTrajectoryVisual): ")

    
    # 创建一个TCP套接字
    # 这一行创建了一个套接字（socket），并将其赋值给变量 server。
    # socket.socket() 函数用于创建套接字对象，AF_INET 参数指定了使用IPv4地址族，SOCK_STREAM 参数指定了使用TCP协议。
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8888)) # 绑定到本地主机（localhost）的8888端口。可以随意更改，客户端和服务端需要使用相同的端口号。
    server.listen(5) # 监听传入的连接，参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。
    connection, address = server.accept()
    columns = ['x', 'y', 'speed', 'headingAngle', 'wheelAngle']
    for i in range(20):
        columns.append('x' + str(i))
        columns.append('y' + str(i))
    vehicle_data = pd.DataFrame(columns=columns)
    boundryx = []
    boundry_sensory = []
    boundry1 = []
    boundry2 = []
    boundry3 = []
    if visual_mode == '0':
        fig, axs = plt.subplots(5, 1, figsize=(10, 8))
    else:
        fig, ax = plt.subplots(1, 1,figsize=(15, 2))
    while True:
        msg = receive_messages(connection)
        
        vehicle_data.loc[len(vehicle_data)] = msg[0]
        obstacle_data = []
        refer_line_data_x = []
        refer_line_data_y = []
        for i in range(1, len(msg) - 1):
            obstacle_data.append(msg[i])
        for i in range(len(msg[-1])):
            if i % 2 == 0:
                refer_line_data_x.append(msg[-1][i])
            else:
                refer_line_data_y.append(msg[-1][i])
        
        #boundry
        if len(boundryx) == 0:
            boundryx.append(vehicle_data.iloc[0, 0])
        else:
            while boundryx[-1] < vehicle_data.iloc[-1, 0] + 25:
                boundryx.append(boundryx[-1] + 0.1)
        if len(boundry_sensory) == 0:
            boundry_sensory.append(vehicle_data.iloc[0, 0])
        else:
            while boundry_sensory[-1] < vehicle_data.iloc[-1, 0] + 150:
                boundry_sensory.append(boundry_sensory[-1] + 0.1)
            while boundry_sensory[0] < vehicle_data.iloc[-1, 0]:
                boundry_sensory.pop(0)

        boundry1 = [0] * len(boundryx)
        boundry2 = [3.5] * len(boundryx)
        boundry3 = [7] * len(boundryx)


        # 全状态显示
        def FullMsgVisual():
            axs[0].cla()  # 清空子图
            axs[1].cla()  
            axs[2].cla()  
            axs[3].cla()  
            axs[4].cla()  

            """绘制车辆轨迹"""
            axs[0].plot(vehicle_data.iloc[:, 0], vehicle_data.iloc[:, 1],'-b') # 行驶轨迹
            axs[0].plot(boundryx, boundry1, color='k') 
            axs[0].plot(boundryx, boundry2, color='k')
            axs[0].plot(boundryx, boundry3, color='k')
            axs[0].set_title('Trajectory')
            axs[0].set_xlabel('X(m)')
            axs[0].set_ylabel('y(m)')

            """绘制车辆姿态"""
            """以下顶点计算方法适用于小转向角"""
            axs[1].plot(boundry_sensory, [0] * len(boundry_sensory), color='k') 
            axs[1].plot(boundry_sensory, [3.5] * len(boundry_sensory), color='k')
            axs[1].plot(boundry_sensory, [7] * len(boundry_sensory), color='k')
            dia_angle = delta + vehicle_data.iloc[-1, 3]
            lower_left = (vehicle_data.iloc[-1, 0] - half_diagonal * math.cos(dia_angle), 
                        vehicle_data.iloc[-1, 1] - half_diagonal * math.sin(dia_angle)) # 左下顶点
            lower_right = (lower_left[0] + vehicle_length * math.cos(vehicle_data.iloc[-1, 3]), 
                        lower_left[1] + vehicle_length * math.sin(vehicle_data.iloc[-1, 3])) # 右下顶点
            upper_left = (lower_left[0] - vehicle_width * math.sin(vehicle_data.iloc[-1, 3]), 
                        lower_left[1] + vehicle_width * math.cos(vehicle_data.iloc[-1, 3]))
            upper_right = (lower_left[0] + 2 * half_diagonal * math.cos(dia_angle), 
                        lower_left[1] + 2 * half_diagonal * math.sin(dia_angle))
            rect = patches.Polygon([lower_left, lower_right, upper_right, upper_left], closed=True, edgecolor='r', facecolor='none')
            axs[1].add_patch(rect)  # 将矩形添加到当前子图中
            for obstacle in obstacle_data:
                rect = patches.Polygon([(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), 
                                        (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])], 
                                        closed=True, edgecolor='r', facecolor='none')
                axs[1].add_patch(rect)
            # 绘制规划轨迹和参考线
            planning_trajectory_x = []
            planning_trajectory_y = []
            for i in range(20):
                planning_trajectory_x.append(vehicle_data.iloc[-1, 5 + 2 * i])
                planning_trajectory_y.append(vehicle_data.iloc[-1, 6 + 2 * i])
            axs[1].plot(planning_trajectory_x, planning_trajectory_y, color='b')
            axs[1].plot(refer_line_data_x, refer_line_data_y, color='g')

            axs[1].set_title('Realtime Position(m)')
            axs[1].set_xlabel('X(m)')
            axs[1].set_ylabel('y(m)')

            """绘制车辆速度"""
            axs[2].plot(vehicle_data.iloc[:, 2], '-b') # 行驶轨迹
            axs[2].set_title('Realtime Speed')
            axs[2].set_xlabel('Time(0.1s)')
            axs[2].set_ylabel('Speed(m/s)')

            """绘制车辆航向角"""
            axs[3].plot(vehicle_data.iloc[:, 3], '-b') 
            axs[3].set_title('Realtime Heading Angle')
            axs[3].set_xlabel('Time(0.1s)')
            axs[3].set_ylabel('Angle(rad)')

            """绘制车辆轮胎转角"""
            axs[4].plot(vehicle_data.iloc[:, 4], '-b')
            axs[4].set_title('Realtime Wheel Angle')
            axs[4].set_xlabel('Time(0.1s)')
            axs[4].set_ylabel('Angle(rad)')
            
            plt.pause(0.01)
            plt.tight_layout()
            
        
        
        # 实时参考线、规划轨迹、障碍物显示
        def PlanningTrajectoryVisual():
            ax.cla()
            """绘制车辆姿态"""
            """以下顶点计算方法适用于小转向角"""
            ax.plot(boundry_sensory, [0] * len(boundry_sensory), color='k') 
            ax.plot(boundry_sensory, [3.5] * len(boundry_sensory), color='k')
            ax.plot(boundry_sensory, [7] * len(boundry_sensory), color='k')
            dia_angle = delta + vehicle_data.iloc[-1, 3]
            lower_left = (vehicle_data.iloc[-1, 0] - half_diagonal * math.cos(dia_angle), 
                        vehicle_data.iloc[-1, 1] - half_diagonal * math.sin(dia_angle)) # 左下顶点
            lower_right = (lower_left[0] + vehicle_length * math.cos(vehicle_data.iloc[-1, 3]), 
                        lower_left[1] + vehicle_length * math.sin(vehicle_data.iloc[-1, 3])) # 右下顶点
            upper_left = (lower_left[0] - vehicle_width * math.sin(vehicle_data.iloc[-1, 3]), 
                        lower_left[1] + vehicle_width * math.cos(vehicle_data.iloc[-1, 3]))
            upper_right = (lower_left[0] + 2 * half_diagonal * math.cos(dia_angle), 
                        lower_left[1] + 2 * half_diagonal * math.sin(dia_angle))
            rect = patches.Polygon([lower_left, lower_right, upper_right, upper_left], closed=True, edgecolor='r', facecolor='none')
            ax.add_patch(rect)  # 将矩形添加到当前子图中
            for obstacle in obstacle_data:
                rect = patches.Polygon([(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), 
                                        (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])], 
                                        closed=True, edgecolor='r', facecolor='none')
                ax.add_patch(rect)
            # 绘制规划轨迹和参考线
            planning_trajectory_x = []
            planning_trajectory_y = []
            for i in range(20):
                planning_trajectory_x.append(vehicle_data.iloc[-1, 5 + 2 * i])
                planning_trajectory_y.append(vehicle_data.iloc[-1, 6 + 2 * i])
            ax.plot(planning_trajectory_x, planning_trajectory_y, color='b')
            ax.plot(refer_line_data_x, refer_line_data_y, color='g')

            ax.set_title('Realtime Position(m)')
            ax.set_xlabel('X(m)')
            ax.set_ylabel('y(m)')
            plt.pause(0.01)
            plt.tight_layout()
        
        if visual_mode == '0':
            FullMsgVisual()
        elif visual_mode == '1':
            PlanningTrajectoryVisual()

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