import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import pandas as pd
import os


def LonVisual(data_s, data_v, data_a):
    script_directory = os.path.dirname(os.path.realpath(__file__))
    # 构建相对路径
    csv_path_s = os.path.join(script_directory, '..', 'data', data_s)
    csv_path_v = os.path.join(script_directory, '..', 'data', data_v)
    csv_path_a = os.path.join(script_directory, '..', 'data', data_a)
    # 读取CSV文件
    df_s = pd.read_csv(csv_path_s)
    df_v = pd.read_csv(csv_path_v)
    df_a = pd.read_csv(csv_path_a)

    plt.subplot(3, 1, 1)
    plt.plot(df_s, linestyle='-', color='b', label='s')
    plt.title('s')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Distance(m)')

    ax2 = plt.subplot(3, 1, 2)
    plt.plot(df_v, linestyle='-', color='r', label='v')
    plt.title('v(target speed is 15m/s)')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Speed(m/s)')
    ax2.set_ylim(0, 30)
    ax2.yaxis.set_major_locator(MultipleLocator(base=5))

    ax3 = plt.subplot(3, 1, 3)
    plt.plot(df_a, linestyle='-', color='y', label='a')
    plt.title('a')
    plt.xlabel('Step(0.1s)')
    plt.ylabel('Acceleration(m)')
    ax3.set_ylim(-7, 5)
    ax3.yaxis.set_major_locator(MultipleLocator(base=2))

    # 调整布局
    plt.tight_layout()

    # 显示图形
    plt.show()


LonVisual("s.csv", "v.csv", "a.csv")