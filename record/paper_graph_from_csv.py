#!/usr/bin/env python3
import argparse
import rosbag2_py
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
from Cluster import Cluster
import numpy as np
from datetime import datetime
from zoneinfo import ZoneInfo

ros_sqlite3 = ["foxy", "galactic", "humble"]

def graph(csv_path):
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    print(data)
    
    ts = data.T[0]
    x_c_des = data.T[9*3+1]
    y_c_des = data.T[9*3+2]
    t_c_des = data.T[9*3+3]
    p_des = data.T[9*3+7]
    q_des = data.T[9*3+8]
    B_des = data.T[9*3+9]

    x_c = data.T[9*2+1]
    y_c = data.T[9*2+2]
    t_c = data.T[9*2+3]
    p = data.T[9*2+7]
    q = data.T[9*2+8]
    B = data.T[9*2+9]

    length = 600*3
    start = 0
    end = length *4

    os.makedirs("figures", exist_ok=True)
    fig = plt.figure()
    plt.plot(x_c_des[start:end], y_c_des[start:end], label="Cluster Desired")
    plt.plot(x_c[start:end], y_c[start:end], label="Cluster")

    plt.title("x_c vs y_c")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    fig.savefig(f"figures/cluster.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], x_c_des[start:end], label="desired x_c")
    plt.plot(ts[start:end], x_c[start:end], label="actual x_c")
    plt.title("x_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("x_c")
    plt.legend()
    fig.savefig(f"figures/x_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], y_c_des[start:end], label="desired y_c")
    plt.plot(ts[start:end], y_c[start:end], label="actual y_c")
    plt.title("y_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("y_c")
    plt.legend()
    fig.savefig(f"figures/y_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], t_c_des[start:end], label="desired t_c")
    plt.plot(ts[start:end], t_c[start:end], label="actual t_c")
    plt.title("t_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("t_c")
    plt.legend()
    fig.savefig(f"figures/y_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], p_des[start:end], label="desired p")
    plt.plot(ts[start:end], p[start:end], label="actual p")
    plt.title("p vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("p")
    plt.legend()
    fig.savefig(f"figures/p.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], q_des[start:end], label="desired q")
    plt.plot(ts[start:end], q[start:end], label="actual q")
    plt.title("q vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("q")
    plt.legend()
    fig.savefig(f"figures/q.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], B_des[start:end], label="desired B")
    plt.plot(ts[start:end], B[start:end], label="actual B")
    plt.title("B vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("B")
    plt.legend()
    fig.savefig(f"figures/B.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 bag reader"
    )

    parser.add_argument(
        "datacsv", 
        help="datafiles"
    )
    args = parser.parse_args()
    
    graph(args.datacsv)

if __name__ == "__main__":
    main()