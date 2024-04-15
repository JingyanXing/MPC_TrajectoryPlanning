import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import pandas as pd
import os


def VehicleStateVisual():
    script_directory = os.path.dirname(os.path.realpath(__file__))
    # 构建相对路径
    vehicle_state = os.path.join(script_directory, '..', 'data', "vehiclestate.csv")
    refer_line = os.path.join(script_directory, '..', 'data', "latSolverUnitTest_refer_lane.csv")
    # 读取CSV文件
    df_v = pd.read_csv(vehicle_state)
    df_l = pd.read_csv(refer_line)

    plt.figure(figsize=(10, 8))
    #pos
    boundary_x = [0]
    while(boundary_x[-1] < df_v.iloc[-1,0]):
        boundary_x.append(boundary_x[-1] + 1)
    plt.subplot(5, 1, 1)
    plt.plot(df_v.iloc[:,0], df_v.iloc[:,1], '.', label='real position', color='b')
    # plt.plot(df_l.iloc[:, 0], df_l.iloc[:,1], '.', label='target position', color='r')
    plt.plot(boundary_x, [0] * len(boundary_x), linestyle='-', color='k')
    plt.plot(boundary_x, [3.5] * len(boundary_x), linestyle='-', color='k')
    plt.plot(boundary_x, [7] * len(boundary_x), linestyle='-', color='k')
    plt.title('Position(m)')
    plt.xlabel('X(m)')
    plt.ylabel('y(m)')
    plt.legend()

    # speed
    ax2 = plt.subplot(5, 1, 2)
    plt.plot(df_v.iloc[:,2], linestyle='-', color='b')
    plt.plot(df_v.iloc[:,4], linestyle='-', color='r', label='target speed')
    plt.title('Speed(m/s)')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Speed(m/s)')
    plt.legend()

    # acc
    ax3 = plt.subplot(5, 1, 3)
    plt.plot(df_v.iloc[:,3], linestyle='-', color='b', label='real acceleration')
    plt.title('Acceleration(m/s^2)')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Acceleration(m)')
    plt.legend(loc = 'lower right')

    # heading
    ax4 = plt.subplot(5, 1, 4)
    plt.plot(df_v.iloc[:,5], linestyle='-', color='b', label='real heading angle')
    plt.title('Heading angle(rad)')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Heading angle(rad)')
    plt.legend()

    # wheel angle
    ax5 = plt.subplot(5, 1, 5)
    plt.plot(df_v.iloc[:,6], linestyle='-', color='b', label='real wheel angle')
    plt.title('Wheel angle(rad)')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Wheel angle(rad)')
    plt.legend()

    # 调整布局
    plt.tight_layout()

    plt.subplots_adjust(hspace=1.5)

    # 显示图形
    plt.show()


VehicleStateVisual()